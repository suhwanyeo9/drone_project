"""
obstacle_locator_multi_node.py — 좌표 계산 노드 (Phase 4 다중 목표판) v2
=======================================================================
obstacle_locator_node.py (정우열) 를 import 해 상속한다.
투영 수학·TF 조회·timeout 등 검증된 로직은 전부 원본 것을 그대로 쓰고,
아래 변경점만 얹는다. 원본 파일은 수정하지 않는다.

[원본의 구조적 한계 — Step 1 검증에서 실측]
  원본은 탐지 픽셀을 self.pending "하나"에 저장했다가 타이머가 투영한다.
  다중 목표 월드에서 탐지 3건이 연달아 도착하면 앞의 2건이 덮어써져,
  항상 마지막 목표 하나만 투영된다 (실측: target 1 만 확정, 2·3 은 유실).
  단일 목표 전제에서는 문제가 없었으므로 버그가 아니라 설계 범위의 차이다.

[변경점 3가지]
  1. on_detection_pixel 오버라이드 — 탐지 즉시 투영:
     탐지가 도착한 그 자리에서 '그 영상 시각의 드론 pose' 로 투영해
     발행한다. 저장 단계가 없으므로 덮어쓰기가 구조적으로 불가능하다.
     ※ Phase 3 규약("탐지 = 시각이 기록된 1회성 사건") 유지.

  2. 초기 유령 목표 제거:
     원본은 시작 직후 detect_pixel 기본값(화면 중앙 320,240)을 투영해
     드론 바로 아래가 목표로 잡힌다 (실측: map(+0.01, +0.01)).
     실제 탐지가 올 때까지 아무것도 하지 않도록 pending 을 비운다.

  3. 비블로킹 재시도 큐 (v2 — 실측으로 발견한 문제의 해결):
     [문제] 콜백 안에서 lookup_timeout(100ms) 대기를 하면, 초당 15건
       (5Hz × 3목표) 탐지에서 대기 시간이 처리량을 초과해 실행자가
       TF 콜백을 처리할 틈을 잃는다 → TF 버퍼가 계속 비어 전부 실패
       (실측: 투영 0건, 실패 400+건 누적).
     [1차 대응] lookup_timeout:=0.0 → 투영은 재개됐으나, 카메라(10Hz)
       영상 시각이 TF 버퍼 최신 시각보다 몇 ms 앞서는 주기적 순간에
       약 25% 실패 잔존 (실측: 259건 성공 / 88건 실패).
       — 선배가 timeout 100ms 를 넣었던 이유가 바로 이 몇 ms 였다.
     [해결] 기다리는 대신 실패 건을 큐에 쟁여두고, 다음 탐지 콜백과
       주기 타이머에서 재시도한다. 몇 ms 뒤엔 TF 가 도착해 있으므로
       거의 전부 성공하고, 블로킹이 없어 기아도 재발하지 않는다.
       0.5초 넘게 실패하는 건은 폐기 (탐지기가 5Hz 로 계속 보내므로
       한 건 잃어도 같은 목표가 곧 다시 들어온다).
     → 실행 시 -p lookup_timeout:=0.0 권장 (기본 100ms 를 끄는 것).

[출력 토픽 — 신규]
  /obstacle/ground_points  (geometry_msgs/PointStamped)
    point.x, point.y : 확정된 map 좌표
    header.stamp     : 탐지에 쓰인 영상의 촬영 시각 (규약 유지)
    header.frame_id  : world_frame (기본 map)
  한 프레임에서 목표 N개가 검출되면 같은 stamp 로 N건이 발행된다.
  → Step 2 의 target_manager_node 가 이 토픽을 구독해 클러스터링한다.
  기존 단일 목표 토픽(/obstacle/ground_point 등)은 건드리지 않는다.

사용:
  python3 obstacle_locator_multi_node.py --ros-args \
    -p use_tf2:=true -p world_frame:=map \
    -p camera_frame:=drone_camera_optical -p robot_frame:=base_link \
    -p use_sim_time:=true -p lookup_timeout:=0.0

담당: 여수환 · 스마트해운물류 x ICT 멘토링 (파트 3) · Phase 4
"""
import os
import sys
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

# ── 원본 모듈 import (같은 디렉토리의 obstacle_locator_node.py) ──
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import obstacle_locator_node as base  # noqa: E402

# 원본의 노드 클래스를 이름에 의존하지 않고 동적으로 찾는다
# (선배가 클래스명을 바꿔도 깨지지 않도록).
_BaseNode = None
for _name in dir(base):
    _obj = getattr(base, _name)
    if isinstance(_obj, type) and issubclass(_obj, Node) and _obj is not Node:
        _BaseNode = _obj
        break
if _BaseNode is None:
    raise ImportError("obstacle_locator_node.py 에서 Node 클래스를 찾지 못했습니다.")

RETRY_MAX_AGE = 0.5   # [s] 이보다 오래 실패한 탐지는 폐기
RETRY_MAX_LEN = 60    # 큐 상한 (약 4초분) — 폭주 방지


class ObstacleLocatorMultiNode(_BaseNode):
    def __init__(self):
        super().__init__()

        # ── [변경 2] 초기 유령 목표 제거 ──
        self.pending = None

        # ── [변경 3] 재시도 큐: (u, v, stamp, 접수시각) ──
        self.retry_q = deque(maxlen=RETRY_MAX_LEN)

        # ── 신규 출력: 다중 목표 좌표 스트림 ──
        self.pub_ground_points = self.create_publisher(
            PointStamped, "/obstacle/ground_points", 10)

        self.n_projected = 0
        self.n_dropped = 0
        # 원본 타이머(rate_hz, 기본 2Hz)에 재시도 처리를 얹는다.
        self.retry_timer = self.create_timer(0.1, self._drain_retry_queue)

        self.get_logger().info(
            "다중 목표 모드 시작 — 탐지 즉시 투영 + 재시도 큐, "
            "/obstacle/ground_points 발행 (초기 유령 목표 제거됨)")

    # ─────────────────────────────────────────────
    def _try_project_and_publish(self, u, v, stamp):
        """1건 투영 시도. 성공하면 발행하고 True, TF 미도착이면 False."""
        lookup_stamp = None
        if self.use_detection_stamp:
            lookup_stamp = rclpy.time.Time.from_msg(stamp)

        cam = self.get_camera_R_C(lookup_stamp)
        if cam is None:
            return False

        R_cam, C_cam = cam
        try:
            distance, bearing, P = base.pixel_to_ground(u, v, self.K, R_cam, C_cam)
        except ValueError as e:
            # 기하학적으로 투영 불가(광선이 지면과 안 만남 등) — 재시도 무의미, 폐기
            self.get_logger().warn(f"투영 실패(폐기): {e}", throttle_duration_sec=2.0)
            return True  # 처리 완료로 간주 (큐에 남기지 않음)

        out = PointStamped()
        out.header.stamp = stamp                  # ★ 규약: 영상 촬영 시각 유지
        out.header.frame_id = self.world_frame
        out.point.x = float(P[0])
        out.point.y = float(P[1])
        out.point.z = 0.0
        self.pub_ground_points.publish(out)
        self.n_projected += 1

        self.get_logger().info(
            f"투영 발행 픽셀({u:.0f},{v:.0f}) → map({P[0]:+.3f}, {P[1]:+.3f}) "
            f"| 드론 ({C_cam[0]:+.2f}, {C_cam[1]:+.2f}, {C_cam[2]:+.2f}) "
            f"| 누적 {self.n_projected}건 (폐기 {self.n_dropped})",
            throttle_duration_sec=1.0)
        return True

    # ─────────────────────────────────────────────
    def _drain_retry_queue(self):
        """큐에 쟁여둔 실패 건들을 재시도. 오래된 건 폐기."""
        now = self.get_clock().now()
        remaining = deque(maxlen=RETRY_MAX_LEN)
        while self.retry_q:
            u, v, stamp, recv = self.retry_q.popleft()
            age = (now - recv).nanoseconds * 1e-9
            if age > RETRY_MAX_AGE:
                self.n_dropped += 1
                continue
            if not self._try_project_and_publish(u, v, stamp):
                remaining.append((u, v, stamp, recv))
        self.retry_q = remaining

    # ─────────────────────────────────────────────
    def on_detection_pixel(self, msg: PointStamped):
        """[변경 1+3] 탐지 즉시 투영, 실패 시 재시도 큐에 적재 (블로킹 없음)."""
        u, v = float(msg.point.x), float(msg.point.y)
        stamp = msg.header.stamp

        # 밀린 것 먼저 처리 (도착 순서 보존)
        self._drain_retry_queue()

        if not self._try_project_and_publish(u, v, stamp):
            self.retry_q.append((u, v, stamp, self.get_clock().now()))


def main():
    rclpy.init()
    node = ObstacleLocatorMultiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
