"""
target_manager_node.py — 다중 목표 클러스터링·확정 관리 (Phase 4 Step 2) v2
=============================================================================
드론이 비행하며 같은 목표를 수십~수백 번 재탐지하므로,
낱개 좌표 스트림을 "타겟 N개의 확정 리스트" 로 정리하는 노드.
v3: /targets/reset (std_msgs/Empty) 수신 시 타겟·후보 전부 초기화.
    시뮬을 껐다 켜지 않고 시나리오를 다시 돌릴 때 사용:
      ros2 topic pub --once /targets/reset std_msgs/msg/Empty "{}"
    (탐사 대기 플래그도 초기화 — 다음 탐사 시작을 다시 기다린다)
v2: require_survey(기본 true) — 드론 탐사(drone_survey)가 시작된 뒤에만
    타겟을 등록한다. /drone/survey_state 가 1건이라도 수신되면 탐사 중으로
    간주. 통합 데모에서 "드론이 탐사하며 발견 → 로봇 출발" 순서를 보장하고,
    탐사 전 정지 상태에서 우연히 보이는 목표가 미리 확정되는 것을 막는다.
    (개별 테스트에서 예전처럼 즉시 등록하려면 -p require_survey:=false)

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
from std_msgs.msg import String, Empty

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
        self.declare_parameter("require_survey", True)  # [v2] 탐사 시작 후에만 등록

        g = lambda n: self.get_parameter(n).value
        self.merge_radius = float(g("merge_radius"))
        self.confirm_count = int(g("confirm_count"))
        rate = float(g("publish_rate_hz"))
        self.world_frame = str(g("world_frame"))
        self.require_survey = bool(g("require_survey"))
        self.survey_seen = False   # [v2] /drone/survey_state 수신 여부

        self.targets: list[Target] = []
        self.n_points = 0
        self._last_confirmed_n = 0   # 확정 수가 변할 때만 알림 로그

        self.create_subscription(PointStamped, "/obstacle/ground_points",
                                 self.on_ground_point, 50)
        # [v2] 탐사 상태 구독 — 어떤 값이든 1건 수신되면 탐사 시작으로 간주
        self.create_subscription(String, "/drone/survey_state",
                                 self.on_survey_state, 10)
        # [v3] 리셋 구독 — 시뮬 재시작 없이 시나리오 재실행용
        self.create_subscription(Empty, "/targets/reset", self.on_reset, 10)
        self.pub_confirmed = self.create_publisher(PoseArray, "/targets/confirmed", 10)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(
            f"target_manager 시작 | merge_radius={self.merge_radius}m, "
            f"confirm_count={self.confirm_count}, 발행 {rate}Hz, "
            f"탐사 대기 {'ON' if self.require_survey else 'OFF'}")

    # ─────────────────────────────────────────────
    def on_reset(self, msg: Empty):
        """[v3] 타겟·후보·탐사 플래그 전부 초기화."""
        n = len(self.targets)
        self.targets = []
        self.n_points = 0
        self.survey_seen = False
        self.get_logger().info(
            f"◇ 리셋 — 타겟 {n}개 삭제, 다음 탐사 시작 대기")

    # ─────────────────────────────────────────────
    def on_survey_state(self, msg: String):
        """[v2] 탐사 노드가 살아있다는 신호."""
        if not self.survey_seen:
            self.survey_seen = True
            self.get_logger().info("드론 탐사 감지 — 타겟 등록을 시작합니다")

    # ─────────────────────────────────────────────
    def on_ground_point(self, msg: PointStamped):
        # [v2] 탐사 시작 전에는 좌표를 받아들이지 않는다
        if self.require_survey and not self.survey_seen:
            self.get_logger().info(
                "탐사 시작 전 — 탐지 무시 중 (드론 survey 대기)",
                throttle_duration_sec=10.0)
            return
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
