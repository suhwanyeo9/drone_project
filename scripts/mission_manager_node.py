"""
mission_manager_node.py — 다중 목표 순차 방문 관리 (Phase 4 Step 3)
====================================================================
target_manager 가 확정한 타겟들을 로봇이 가까운 순서대로 방문하게 한다.

[입력]
  /targets/confirmed (geometry_msgs/PoseArray, map 프레임)
    target_manager_node 가 1Hz 로 발행하는 확정 타겟 리스트.

[출력]
  Nav2 의 NavigateToPose "액션" 으로 goal 전송.
  ┌ 왜 /goal_pose 토픽(기존 goal_relay 방식)이 아니라 액션인가:
  │   토픽은 단방향이라 "도착했는지" 를 알 수 없다. 목표가 1개일 땐
  │   상관없지만, 다중 목표 순회는 도착을 알아야 다음 goal 을 쏠 수
  │   있다. 액션은 goal 전송 → 수락 → 결과(성공/실패/취소) 회신이
  └   한 세트라 순차 방문의 상태 기계를 만들 수 있다.

[동작 — 상태 기계]
  WAITING  : 확정 타겟이 min_targets(기본 3)개 모일 때까지 대기
  SELECT   : 미방문 타겟 중 로봇 현재 위치에서 가장 가까운 것 선택 (greedy)
             ┌ greedy 로 충분한 근거: 타겟 수가 적어(3개) 최적 경로(TSP)
             └ 와의 차이가 미미하다. 대규모 확장 시 TSP 고려로 보고.
  NAVIGATE : NavigateToPose goal 전송, 결과 대기
  → 성공   : 방문 처리 후 SELECT 로 (남은 타겟이 있으면)
  → 실패   : 재시도 카운트 후 max_retries 초과 시 해당 타겟 포기(skip)
  DONE     : 전부 방문(또는 포기) → 미션 완료 로그, 소요 시간 출력

[도착 판정]
  Nav2 의 goal 성공 회신을 그대로 쓴다. 단, 타겟 좌표는 기둥 "중심" 이고
  기둥에는 collision 이 있어 로봇이 중심까지 물리적으로 못 들어간다.
  → goal 을 타겟 중심에서 approach_offset(기본 0.6m)만큼 로봇 쪽으로
    당긴 지점으로 보낸다 (기둥 반지름 0.3m + 여유).
    goal 자세(yaw)는 타겟을 바라보는 방향으로 설정.

사용:
  python3 mission_manager_node.py --ros-args -p use_sim_time:=true
  # 파라미터: -p min_targets:=3 -p approach_offset:=0.6 -p max_retries:=1

담당: 여수환 · 스마트해운물류 x ICT 멘토링 (파트 3) · Phase 4
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import NavigateToPose

try:
    from tf2_ros import Buffer, TransformListener
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False


def yaw_to_quat(yaw):
    """z축 회전(yaw)만 있는 쿼터니언 (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__("mission_manager_node")

        self.declare_parameter("min_targets", 3)       # 미션 시작에 필요한 확정 타겟 수
        self.declare_parameter("approach_offset", 0.6)  # [m] 타겟 중심 앞 정지 거리
        self.declare_parameter("max_retries", 1)       # 타겟당 재시도 횟수
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("robot_frame", "base_link")

        g = lambda n: self.get_parameter(n).value
        self.min_targets = int(g("min_targets"))
        self.approach_offset = float(g("approach_offset"))
        self.max_retries = int(g("max_retries"))
        self.world_frame = str(g("world_frame"))
        self.robot_frame = str(g("robot_frame"))

        if not TF2_AVAILABLE:
            raise RuntimeError("tf2_ros 가 필요합니다 (로봇 현재 위치 조회용).")
        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, self)  # noqa: F841

        self.create_subscription(PoseArray, "/targets/confirmed",
                                 self.on_targets, 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── 미션 상태 ──
        self.targets = []          # [(x, y), ...] 확정 타겟 (미션 시작 시 고정)
        self.visited = []          # [bool, ...]
        self.retries = []          # [int, ...]
        self.state = "WAITING"     # WAITING → NAVIGATE(반복) → DONE
        self.current_idx = None
        self.mission_start = None
        self._goal_handle = None

        self.timer = self.create_timer(1.0, self.on_timer)
        self.get_logger().info(
            f"mission_manager 시작 | 시작 조건: 확정 타겟 {self.min_targets}개, "
            f"접근 오프셋 {self.approach_offset}m, 재시도 {self.max_retries}회")

    # ─────────────────────────────────────────────
    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.robot_frame, rclpy.time.Time())
            t = tf.transform.translation
            return (t.x, t.y)
        except Exception:
            return None

    # ─────────────────────────────────────────────
    def on_targets(self, msg: PoseArray):
        """확정 타겟 리스트 갱신. 미션 시작 전에만 반영한다.
        (미션 중에는 목표가 바뀌지 않도록 스냅샷을 고정 —
         방문 도중 리스트가 흔들리면 순회 논리가 꼬인다.)"""
        if self.state != "WAITING":
            return
        self.targets = [(p.position.x, p.position.y) for p in msg.poses]

    # ─────────────────────────────────────────────
    def on_timer(self):
        if self.state == "WAITING":
            n = len(self.targets)
            self.get_logger().info(
                f"[WAITING] 확정 타겟 {n}/{self.min_targets} 대기 중",
                throttle_duration_sec=5.0)
            if n >= self.min_targets:
                self.visited = [False] * len(self.targets)
                self.retries = [0] * len(self.targets)
                self.mission_start = self.get_clock().now()
                tlist = ", ".join(f"({x:+.2f},{y:+.2f})" for x, y in self.targets)
                self.get_logger().info(
                    f"■ 미션 시작 — 타겟 {len(self.targets)}개 고정: {tlist}")
                self.state = "SELECT"

        elif self.state == "SELECT":
            self._select_and_go()

        # NAVIGATE 중에는 액션 콜백이 상태를 넘겨주므로 타이머는 대기.
        # DONE 이면 아무것도 안 함.

    # ─────────────────────────────────────────────
    def _select_and_go(self):
        # 남은 타겟?
        remaining = [i for i, v in enumerate(self.visited) if not v]
        if not remaining:
            dt = (self.get_clock().now() - self.mission_start).nanoseconds * 1e-9
            ok = sum(1 for i, v in enumerate(self.visited)
                     if v and self.retries[i] <= self.max_retries)
            self.get_logger().info(
                f"■■ 미션 완료 — {len(self.targets)}개 중 방문 처리 {ok}개, "
                f"소요 {dt:.1f}초")
            self.state = "DONE"
            return

        robot = self._robot_xy()
        if robot is None:
            self.get_logger().warn("로봇 TF 조회 실패 — 1초 후 재시도",
                                   throttle_duration_sec=5.0)
            return
        rx, ry = robot

        # greedy: 가장 가까운 미방문 타겟
        idx = min(remaining,
                  key=lambda i: math.hypot(self.targets[i][0] - rx,
                                           self.targets[i][1] - ry))
        tx, ty = self.targets[idx]
        self.current_idx = idx

        # 접근 지점: 타겟 중심에서 로봇 방향으로 offset 만큼 당김
        d = math.hypot(tx - rx, ty - ry)
        if d > 1e-6:
            gx = tx - (tx - rx) / d * self.approach_offset
            gy = ty - (ty - ry) / d * self.approach_offset
        else:
            gx, gy = tx, ty
        yaw = math.atan2(ty - gy, tx - gx)   # 타겟을 바라보는 방향

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.world_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        qx, qy, qz, qw = yaw_to_quat(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 액션 서버 없음 — 1초 후 재시도")
            return

        self.get_logger().info(
            f"▶ 타겟 #{idx + 1} ({tx:+.2f},{ty:+.2f}) 로 출발 — "
            f"goal ({gx:+.2f},{gy:+.2f}), 로봇에서 {d:.2f}m")
        self.state = "NAVIGATE"
        send = self.nav_client.send_goal_async(goal)
        send.add_done_callback(self._on_goal_response)

    # ─────────────────────────────────────────────
    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f"타겟 #{self.current_idx + 1} goal 거부됨 — 재선택")
            self._register_failure()
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        idx = self.current_idx
        status = future.result().status
        # action_msgs/GoalStatus: 4 = SUCCEEDED
        if status == 4:
            self.visited[idx] = True
            done = sum(self.visited)
            self.get_logger().info(
                f"✔ 타겟 #{idx + 1} 도착 ({done}/{len(self.targets)})")
            self.state = "SELECT"
        else:
            self.get_logger().warn(
                f"✘ 타겟 #{idx + 1} 주행 실패 (status={status})")
            self._register_failure()

    def _register_failure(self):
        idx = self.current_idx
        self.retries[idx] += 1
        if self.retries[idx] > self.max_retries:
            self.visited[idx] = True   # 방문 처리(포기)로 표시해 순회에서 제외
            self.get_logger().warn(
                f"타겟 #{idx + 1} 재시도 초과 — 포기하고 다음 타겟으로")
        self.state = "SELECT"


def main():
    rclpy.init()
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
