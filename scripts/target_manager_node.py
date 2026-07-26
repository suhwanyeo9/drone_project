"""
target_manager_node.py — 다중 목표 클러스터링·확정 관리 (Phase 4 Step 2)
=========================================================================
드론이 비행하며 같은 목표를 수십~수백 번 재탐지하므로,
낱개 좌표 스트림을 "타겟 N개의 확정 리스트" 로 정리하는 노드.

[입력]
  /obstacle/ground_points (geometry_msgs/PointStamped)
    obstacle_locator_multi_node 가 발행하는 확정 map 좌표 낱개 스트림.
    (Step 1 실측: 목표 3개 기준 초당 약 15건)

[클러스터링 규칙 — 가이드 4-2]
  새 좌표 P 수신
    → 기존 타겟 중 거리 < merge_radius(기본 0.5m) 인 것이 있으면
       그 타겟에 병합: 좌표를 누적 평균으로 갱신, 관측 횟수 +1
         평균 갱신식:  P̄ₙ = P̄ₙ₋₁ + (P − P̄ₙ₋₁)/n
         (관측이 쌓일수록 개별 탐지의 잡음이 상쇄되어 참값에 수렴)
    → 없으면 새 후보로 등록
  관측 횟수 ≥ confirm_count(기본 3) 인 타겟만 "확정" 으로 승격
    (1회성 오탐이 자동으로 걸러진다)

[merge_radius = 0.5m 선정 근거 — Step 0/1 실측]
  이동 중 탐지 좌표의 산포가 반경 약 0.23~0.48m (Week5 통계 및
  Step 0 재현에서 실측). 0.5m 는 이 산포를 한 클러스터로 덮으면서,
  월드의 목표 간 최소 거리 2.83m 와는 5배 이상 여유가 있다.

[출력]
  /targets/confirmed (geometry_msgs/PoseArray, 1Hz)
    확정 타겟들의 map 좌표. position.x/y 만 사용, orientation 은 단위값.
    header.frame_id = map.
    → Step 3 의 mission_manager 가 이 리스트를 구독해 순차 방문한다.

사용:
  python3 target_manager_node.py --ros-args -p use_sim_time:=true
  # 파라미터 조정 예:
  #   -p merge_radius:=0.5 -p confirm_count:=3

담당: 여수환 · 스마트해운물류 x ICT 멘토링 (파트 3) · Phase 4
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseArray, Pose

MAX_CANDIDATES = 50   # 후보 수 상한 (오탐 폭주로 인한 메모리 증가 방지)


class Target:
    """클러스터 하나 = 타겟 후보 하나."""
    __slots__ = ("x", "y", "n_obs")

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.n_obs = 1

    def merge(self, x, y):
        """누적 평균으로 중심 갱신: P̄ₙ = P̄ₙ₋₁ + (P − P̄ₙ₋₁)/n"""
        self.n_obs += 1
        self.x += (x - self.x) / self.n_obs
        self.y += (y - self.y) / self.n_obs

    def dist_to(self, x, y):
        return math.hypot(self.x - x, self.y - y)


class TargetManagerNode(Node):
    def __init__(self):
        super().__init__("target_manager_node")

        self.declare_parameter("merge_radius", 0.5)    # [m] 병합 반경
        self.declare_parameter("confirm_count", 3)     # 확정 승격 관측 횟수
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("world_frame", "map")

        g = lambda n: self.get_parameter(n).value
        self.merge_radius = float(g("merge_radius"))
        self.confirm_count = int(g("confirm_count"))
        rate = float(g("publish_rate_hz"))
        self.world_frame = str(g("world_frame"))

        self.targets: list[Target] = []
        self.n_points = 0
        self._last_confirmed_n = 0   # 확정 수가 변할 때만 알림 로그

        self.create_subscription(PointStamped, "/obstacle/ground_points",
                                 self.on_ground_point, 50)
        self.pub_confirmed = self.create_publisher(PoseArray, "/targets/confirmed", 10)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(
            f"target_manager 시작 | merge_radius={self.merge_radius}m, "
            f"confirm_count={self.confirm_count}, 발행 {rate}Hz")

    # ─────────────────────────────────────────────
    def on_ground_point(self, msg: PointStamped):
        x, y = float(msg.point.x), float(msg.point.y)
        self.n_points += 1

        # 가장 가까운 기존 타겟 찾기
        best = None
        best_d = float("inf")
        for t in self.targets:
            d = t.dist_to(x, y)
            if d < best_d:
                best, best_d = t, d

        if best is not None and best_d < self.merge_radius:
            was_confirmed = best.n_obs >= self.confirm_count
            best.merge(x, y)
            if not was_confirmed and best.n_obs >= self.confirm_count:
                self.get_logger().info(
                    f"★ 타겟 확정 #{self.targets.index(best) + 1} "
                    f"({best.x:+.3f}, {best.y:+.3f}) — 관측 {best.n_obs}회")
        else:
            if len(self.targets) >= MAX_CANDIDATES:
                self.get_logger().warn(
                    f"후보 상한({MAX_CANDIDATES}) 도달 — 새 후보 무시 "
                    f"({x:+.2f}, {y:+.2f})", throttle_duration_sec=5.0)
                return
            self.targets.append(Target(x, y))
            self.get_logger().info(
                f"새 후보 등록 #{len(self.targets)} ({x:+.3f}, {y:+.3f})")

    # ─────────────────────────────────────────────
    def confirmed(self):
        return [t for t in self.targets if t.n_obs >= self.confirm_count]

    def on_timer(self):
        conf = self.confirmed()

        out = PoseArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.world_frame
        for t in conf:
            p = Pose()
            p.position.x = t.x
            p.position.y = t.y
            p.orientation.w = 1.0
            out.poses.append(p)
        self.pub_confirmed.publish(out)

        # 상태 요약 (5초에 한 번)
        summary = ", ".join(
            f"#{i + 1}({t.x:+.2f},{t.y:+.2f})×{t.n_obs}"
            for i, t in enumerate(self.targets))
        self.get_logger().info(
            f"확정 {len(conf)}/{len(self.targets)} | 수신 {self.n_points}건 | {summary}",
            throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = TargetManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
