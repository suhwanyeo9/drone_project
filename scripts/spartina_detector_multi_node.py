"""
spartina_detector_multi_node.py — 시뮬용 자동 탐지기 (색 기반, 다중 목표)
==========================================================================
spartina_detector_node.py (정우열, 단일 목표) 의 Phase 4 확장판.
기존 파일은 수정하지 않고 신규 파일로 분리 (언제든 원본으로 복귀 가능).

[기존 버전과의 차이 — 정확히 2가지]

  1. 다중 검출: 최대 면적 덩어리 "하나"가 아니라,
     min_area 이상인 "모든" 빨간 덩어리를 각각 발행한다.
     ┌ 이유: 다중 목표 월드에서는 기둥 2개가 한 프레임에 같이 잡히는
     │       순간이 생긴다 (고도 8m 시야 반경 ±4.6m, 목표 간격 2.8m).
     └       기존 코드는 그중 큰 것 하나만 발행해 나머지를 놓친다.

  2. 가장자리 필터: 무게중심이 화면 가장자리 edge_margin_px 이내면 버린다.
     ┌ 이유: 기둥이 화면 밖으로 반쯤 잘려나가는 프레임에서는
     │       보이는 부분만의 무게중심이 계산되어 실제 중심에서 밀린
     │       이상치가 발행된다 (Step 0 재현 시 실측: 정상값 1.9~2.17
     └       분포에서 1.69 로 이탈). 클러스터링 입력 오염 방지.

[유지하는 것 — 선배 인터페이스 규약 그대로]
  토픽 : /detection/pixel (geometry_msgs/PointStamped)
  header.stamp ★ 탐지에 사용한 영상의 header.stamp 를 그대로 복사.
               now() 를 쓰면 안 된다 (Phase 3 동기화 오차 부활).
  point.x = u (픽셀 열), point.y = v (픽셀 행), point.z = 0

  한 프레임에서 N개가 검출되면 같은 header.stamp 로 N개의 메시지가
  연속 발행된다. 하위 노드(locator)는 각 메시지를 독립 사건으로 처리한다.

사용:
  python3 spartina_detector_multi_node.py --ros-args -p use_sim_time:=true
  python3 spartina_detector_multi_node.py --ros-args -p show_window:=true

담당: 여수환 · 스마트해운물류 x ICT 멘토링 (파트 3) · Phase 4
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

WINDOW = "Spartina Detector (multi)"


class SpartinaDetectorMultiNode(Node):
    def __init__(self):
        super().__init__("spartina_detector_multi_node")
        # ── 파라미터 (기존과 동일 + edge_margin_px 추가) ──
        self.declare_parameter("h_low1", 0)      # 빨강 구간 1 (0 쪽)
        self.declare_parameter("h_high1", 10)
        self.declare_parameter("h_low2", 160)    # 빨강 구간 2 (180 쪽)
        self.declare_parameter("h_high2", 180)
        self.declare_parameter("s_min", 100)     # 채도 하한 (회색 장애물 배제)
        self.declare_parameter("v_min", 60)      # 명도 하한 (그림자 배제)
        self.declare_parameter("min_area_px", 60)      # 노이즈 하한
        self.declare_parameter("publish_rate_hz", 5.0)  # 발행 상한 (프레임 단위)
        self.declare_parameter("edge_margin_px", 30)   # ★신규: 가장자리 배제 폭
        self.declare_parameter("show_window", False)

        g = lambda n: self.get_parameter(n).value
        self.h1 = (int(g("h_low1")), int(g("h_high1")))
        self.h2 = (int(g("h_low2")), int(g("h_high2")))
        self.s_min = int(g("s_min"))
        self.v_min = int(g("v_min"))
        self.min_area = int(g("min_area_px"))
        rate = float(g("publish_rate_hz"))
        self.min_interval = (1.0 / rate) if rate > 0 else 0.0
        self.edge_margin = int(g("edge_margin_px"))
        self.show_window = bool(g("show_window"))

        self.bridge = CvBridge()
        self.last_pub_time = None
        self.n_detect = 0        # 발행한 탐지 수 (개별 목표 단위)
        self.n_edge_skip = 0     # 가장자리 필터로 버린 수
        self.n_frame = 0

        self.create_subscription(Image, "/drone/image_raw", self.on_image, 10)
        self.pub_pixel = self.create_publisher(PointStamped, "/detection/pixel", 10)
        self.pub_debug = self.create_publisher(Image, "/detection/debug_image", 10)
        if self.show_window:
            cv2.namedWindow(WINDOW)
        self.get_logger().info(
            f"spartina_detector_multi 시작 | 빨강 H {self.h1}·{self.h2}, "
            f"S≥{self.s_min}, V≥{self.v_min}, 최소면적 {self.min_area}px, "
            f"발행 {rate}Hz, 가장자리 여유 {self.edge_margin}px"
        )

    # ─────────────────────────────────────────────
    def _find_targets(self, bgr):
        """min_area 이상인 모든 빨간 덩어리의 [(중심픽셀, 면적, 윤곽), ...].
        가장자리 필터는 여기서 하지 않고 on_image 에서 처리한다
        (버린 개수를 통계로 남기고 디버그 화면에 표시하기 위해)."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, (self.h1[0], self.s_min, self.v_min),
                              (self.h1[1], 255, 255))
        m2 = cv2.inRange(hsv, (self.h2[0], self.s_min, self.v_min),
                              (self.h2[1], 255, 255))
        mask = cv2.bitwise_or(m1, m2)
        # 잡티 제거 → 구멍 메우기
        k = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        results = []
        for c in contours:                       # ★변경: max() 1개 → 전부 순회
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            u = M["m10"] / M["m00"]
            v = M["m01"] / M["m00"]
            results.append(((u, v), area, c))
        # 면적 큰 순으로 정렬 (디버그 화면 가독성용, 발행 순서는 의미 없음)
        results.sort(key=lambda r: r[1], reverse=True)
        return results

    # ─────────────────────────────────────────────
    def on_image(self, msg: Image):
        self.n_frame += 1
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]
        found = self._find_targets(frame)

        # ── 발행 속도 제한 (프레임 단위) ──
        # 개별 탐지 단위로 제한하면 한 프레임의 두 번째 목표가 잘려나가므로,
        # "이 프레임을 발행할지"를 통째로 정하고 통과하면 전부 발행한다.
        now = self.get_clock().now()
        frame_ok = True
        if self.min_interval > 0 and self.last_pub_time is not None:
            dt = (now - self.last_pub_time).nanoseconds * 1e-9
            frame_ok = dt >= self.min_interval

        published = 0
        for (u, v), area, contour in found:
            # ── ★신규: 가장자리 필터 ──
            on_edge = (u < self.edge_margin or u > w - self.edge_margin or
                       v < self.edge_margin or v > h - self.edge_margin)
            if on_edge:
                self.n_edge_skip += 1
                # 디버그: 버린 것은 노란 테두리로 구분 표시
                cv2.drawContours(frame, [contour], -1, (0, 255, 255), 2)
                cv2.putText(frame, "edge-skip",
                            (int(u) - 30, int(v) - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                continue

            if frame_ok:
                out = PointStamped()
                # ★ 규약: '지금'이 아니라 이 영상의 header 를 그대로 쓴다.
                out.header = msg.header
                out.point.x, out.point.y, out.point.z = float(u), float(v), 0.0
                self.pub_pixel.publish(out)
                published += 1
                self.n_detect += 1

            # ── 디버그 표시 (유효 탐지: 초록 테두리 + 자홍 십자) ──
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
            cv2.drawMarker(frame, (int(round(u)), int(round(v))), (255, 0, 255),
                           cv2.MARKER_CROSS, 20, 2)
            cv2.putText(frame, f"({u:.0f},{v:.0f}) a={int(area)}",
                        (int(u) - 40, int(v) + 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)

        if frame_ok and published > 0:
            self.last_pub_time = now

        # 상단 상태 표시
        if found:
            cv2.putText(frame, f"targets in frame: {len(found)}  published: {published}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        else:
            cv2.putText(frame, "no target", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # 가장자리 배제 영역을 얇은 회색 사각형으로 표시
        cv2.rectangle(frame, (self.edge_margin, self.edge_margin),
                      (w - self.edge_margin, h - self.edge_margin),
                      (120, 120, 120), 1)

        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
        if self.show_window:
            cv2.imshow(WINDOW, frame)
            cv2.waitKey(1)

        # 100프레임마다 통계 보고
        if self.n_frame % 100 == 0:
            self.get_logger().info(
                f"프레임 {self.n_frame} | 발행 탐지 {self.n_detect}건 | "
                f"가장자리 배제 {self.n_edge_skip}건"
            )


def main():
    rclpy.init()
    node = SpartinaDetectorMultiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
