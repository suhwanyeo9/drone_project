"""
🚁 [드론 코드] drone_survey_once_node.py — 1회 탐사 + 복귀 + 정지 (Phase 4)
===========================================================================
drone_survey_node.py (정우열, 무한 반복 버전) 를 대체하는 Phase 4 버전.
기존 파일은 수정하지 않는다.

[기존 버전과의 차이]
  기존: 지그재그를 무한 반복 (Phase 2 검증용 — 측정 시간을 벌기 위함)
  변경: 지정 범위를 "한 바퀴만" 탐사 → 이륙 지점으로 복귀 → 정지.
        전체 시나리오("탐사 완료 후 로봇 출발")의 전제 조건을 만든다.

[상태 발행 — /drone/survey_state (std_msgs/String)]
  MOVING  : 웨이포인트로 이동 중
  HOLD    : 웨이포인트 정지 중 (hold_sec)
  RETURN  : 탐사 완료, 이륙 지점으로 복귀 중
  DONE    : 복귀 완료, 정지. ← 🚗 mission_manager 가 이 신호를 기다린다.
  ※ target_manager 는 어떤 상태든 1건 수신되면 "탐사 시작" 으로 간주해
    타겟 등록을 시작한다 (기존 인터페이스 유지).

[이동 방식 — 기존과 동일한 원리]
  Gazebo 드론은 kinematic 모델이라 /gazebo/set_entity_state 서비스로
  pose 를 직접 지정해 움직인다. 순간이동이 되지 않도록 타이머(20Hz)로
  매 주기 speed×Δt 만큼만 전진 — TF 가 연속적으로 갱신되어
  이동 중 탐지의 timestamp 동기화(Phase 3)가 유효하게 유지된다.

[지그재그 웨이포인트 생성]
  y 를 y_min 부터 lane_spacing 간격으로 자르고, 레인마다 x 방향을
  번갈아 왕복 (뱀 모양). 레인 간격(3m) < 시야 폭(고도 8m 에서 약 9.2m)
  이므로 빈틈없이 커버된다.

사용:
  python3 drone_survey_once_node.py --ros-args -p use_sim_time:=true \\
    -p area_x_min:=-3.0 -p area_x_max:=3.0 \\
    -p area_y_min:=-3.0 -p area_y_max:=3.0 \\
    -p lane_spacing:=3.0 -p altitude:=8.0 -p speed:=0.8 -p hold_sec:=3.0

담당: 여수환 · 스마트해운물류 x ICT 멘토링 (파트 3) · Phase 4
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState

DRONE_NAME = "drone_camera"
# 카메라가 수직 하방을 보는 자세 (pitch +90°) — 월드 파일과 동일
Q_DOWN = (0.0, 0.7071068, 0.0, 0.7071068)
TICK_HZ = 20.0


class DroneSurveyOnceNode(Node):
    def __init__(self):
        super().__init__("drone_survey_once_node")

        self.declare_parameter("area_x_min", -3.0)
        self.declare_parameter("area_x_max", 3.0)
        self.declare_parameter("area_y_min", -3.0)
        self.declare_parameter("area_y_max", 3.0)
        self.declare_parameter("lane_spacing", 3.0)
        self.declare_parameter("altitude", 8.0)
        self.declare_parameter("speed", 0.8)
        self.declare_parameter("hold_sec", 3.0)

        g = lambda n: float(self.get_parameter(n).value)
        self.x_min, self.x_max = g("area_x_min"), g("area_x_max")
        self.y_min, self.y_max = g("area_y_min"), g("area_y_max")
        self.spacing = g("lane_spacing")
        self.alt = g("altitude")
        self.speed = g("speed")
        self.hold_sec = g("hold_sec")

        # ── 지그재그 웨이포인트 생성 (뱀 모양) ──
        self.waypoints = []
        y = self.y_min
        left_to_right = True
        while y <= self.y_max + 1e-6:
            if left_to_right:
                self.waypoints += [(self.x_min, y), (self.x_max, y)]
            else:
                self.waypoints += [(self.x_max, y), (self.x_min, y)]
            left_to_right = not left_to_right
            y += self.spacing

        # ── 상태 ──
        self.pos = None            # 현재 드론 (x, y) — model_states 로 초기화
        self.home = None           # 이륙 지점 (초기 위치) — 복귀 목적지
        self.wp_idx = 0
        self.state = "MOVING"      # MOVING / HOLD / RETURN / DONE
        self.hold_until = None

        self.pub_state = self.create_publisher(String, "/drone/survey_state", 10)
        self.create_subscription(ModelStates, "/gazebo/model_states",
                                 self.on_model_states, 10)
        self.cli = self.create_client(SetEntityState, "/gazebo/set_entity_state")

        self.timer = self.create_timer(1.0 / TICK_HZ, self.on_tick)
        self.get_logger().info(
            f"drone_survey_once 시작 | 범위 x[{self.x_min},{self.x_max}] "
            f"y[{self.y_min},{self.y_max}], 레인 {self.spacing}m, "
            f"웨이포인트 {len(self.waypoints)}개, 속도 {self.speed}m/s "
            f"— 한 바퀴 후 이륙 지점 복귀·정지")

    # ─────────────────────────────────────────────
    def on_model_states(self, msg: ModelStates):
        """드론 현재 위치 갱신. 최초 수신 시 이륙 지점(home)을 기록한다."""
        try:
            i = list(msg.name).index(DRONE_NAME)
        except ValueError:
            return
        p = msg.pose[i].position
        self.pos = (p.x, p.y)
        if self.home is None:
            self.home = (p.x, p.y)
            self.get_logger().info(
                f"이륙 지점 기록 ({p.x:+.2f}, {p.y:+.2f}) — 탐사 후 여기로 복귀")

    # ─────────────────────────────────────────────
    def _publish_state(self):
        m = String()
        m.data = self.state
        self.pub_state.publish(m)

    def _set_pose(self, x, y):
        """드론 pose 지정 (kinematic 이동)."""
        req = SetEntityState.Request()
        req.state.name = DRONE_NAME
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = self.alt
        (req.state.pose.orientation.x, req.state.pose.orientation.y,
         req.state.pose.orientation.z, req.state.pose.orientation.w) = Q_DOWN
        req.state.reference_frame = "world"
        self.cli.call_async(req)   # 비블로킹 (결과 대기 안 함 — 20Hz 유지)

    # ─────────────────────────────────────────────
    def _current_target(self):
        if self.state == "RETURN":
            return self.home
        if self.wp_idx < len(self.waypoints):
            return self.waypoints[self.wp_idx]
        return None

    def on_tick(self):
        self._publish_state()

        if self.state == "DONE" or self.pos is None:
            return
        if not self.cli.service_is_ready():
            return

        now = self.get_clock().now()

        # ── HOLD: 정지 시간 소화 후 다음 웨이포인트로 ──
        if self.state == "HOLD":
            if now >= self.hold_until:
                self.wp_idx += 1
                if self.wp_idx < len(self.waypoints):
                    self.state = "MOVING"
                    wx, wy = self.waypoints[self.wp_idx]
                    self.get_logger().info(
                        f"→ 웨이포인트 {self.wp_idx + 1}/{len(self.waypoints)} "
                        f"({wx:+.1f}, {wy:+.1f}) 로 이동")
                else:
                    self.state = "RETURN"
                    self.get_logger().info(
                        f"탐사 한 바퀴 완료 — 이륙 지점 "
                        f"({self.home[0]:+.2f}, {self.home[1]:+.2f}) 으로 복귀")
            return

        # ── MOVING / RETURN: 목표로 한 걸음 전진 ──
        target = self._current_target()
        if target is None:
            return
        tx, ty = target
        x, y = self.pos
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        step = self.speed / TICK_HZ

        if dist <= step:
            # 도착
            self._set_pose(tx, ty)
            self.pos = (tx, ty)
            if self.state == "MOVING":
                self.state = "HOLD"
                self.hold_until = now + rclpy.duration.Duration(
                    seconds=self.hold_sec)
                self.get_logger().info(
                    f"  도착 ({tx:+.1f}, {ty:+.1f}) — {self.hold_sec}s 정지")
            else:  # RETURN 완료
                self.state = "DONE"
                self.get_logger().info(
                    "■ 탐사 임무 완료 — 이륙 지점 복귀·정지 (DONE 발행 중)")
        else:
            nx = x + dx / dist * step
            ny = y + dy / dist * step
            self._set_pose(nx, ny)
            self.pos = (nx, ny)


def main():
    rclpy.init()
    node = DroneSurveyOnceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
