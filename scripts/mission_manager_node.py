"""
🚗 [자동차 코드] mission_manager_node.py — 다중 목표 순차 방문 (Phase 4) v4
===========================================================================
target_manager 가 확정한 타겟들을 로봇이 가까운 순서대로 방문하게 한다.
v2: 전 타겟 방문 후 출발 지점으로 복귀하는 RETURN 상태 추가.
v3: 복귀 좌표를 파라미터로 직접 지정 가능 (use_home_pose + home_pose).
v5: /mission/reset (std_msgs/Empty) 수신 시 미션 상태 초기화 → 다시 WAITING.
    시뮬 재시작 없이 시나리오를 다시 돌릴 때 사용.
v4: wait_survey_done(기본 true) — 🚁 드론 탐사가 "DONE" 을 발행할 때까지
    미션 시작을 보류한다. /drone/survey_state 구독.
    시작 조건이 "확정 타겟 N개" 에서 "탐사 완료 + 발견된 타겟 1개 이상" 으로
    바뀐다 — 탐사가 끝나야 발견 목록이 최종본이 되기 때문.
    (기존 방식이 필요하면 -p wait_survey_done:=false → min_targets 기준)

[입력]
  /targets/confirmed (geometry_msgs/PoseArray, map 프레임)
    target_manager_node 가 1Hz 로 발행하는 확정 타겟 리스트.

[출력]
  Nav2 의 NavigateToPose "액션" 으로 goal 전송.
  ┌ 왜 /goal_pose 토픽(기존 goal_relay 방식)이 아니라 액션인가:
  │   토픽은 단방향이라 "도착했는지" 를 알 수 없다. 다중 목표 순회는
  │   도착을 알아야 다음 goal 을 쏠 수 있다. 액션은 goal 전송 → 수락
  └   → 결과(성공/실패) 회신이 한 세트라 순차 방문이 가능하다.

[동작 — 상태 기계]
  WAITING  : 확정 타겟이 min_targets(기본 3)개 모일 때까지 대기.
             이때 로봇의 현재 위치를 "홈" 으로 기록한다.
  SELECT   : 미방문 타겟 중 로봇 현재 위치에서 가장 가까운 것 선택 (greedy)
             ┌ greedy 로 충분한 근거: 타겟 수가 적어(3개) 최적 경로(TSP)
             └ 와의 차이가 미미하다. 대규모 확장 시 TSP 고려로 보고.
  NAVIGATE : NavigateToPose goal 전송, 결과 대기
  → 성공   : 방문 처리 후 SELECT 로
  → 실패   : 재시도 카운트, max_retries 초과 시 해당 타겟 포기(skip)
  RETURN   : [v2 신규] 전 타겟 방문 후 홈으로 복귀 (return_home:=true 일 때)
  DONE     : 미션 완료 로그, 소요 시간 출력

[도착 판정]
  Nav2 의 goal 성공 회신. 타겟 좌표는 기둥 "중심" 이고 기둥엔 collision 이
  있어 로봇이 중심까지 못 들어가므로, goal 을 중심에서 approach_offset
  (기본 0.6m = 기둥 반지름 0.3m + 여유)만큼 로봇 쪽으로 당긴 지점으로 보낸다.
  goal 자세(yaw)는 타겟을 바라보는 방향. 홈 복귀 goal 은 홈 좌표 그대로.

사용:
  python3 mission_manager_node.py --ros-args -p use_sim_time:=true
  # 복귀 끄기: -p return_home:=false
  # [v3] 복귀 좌표 직접 지정:
  #   -p use_home_pose:=true -p home_pose:="[-3.0, -3.0]"
  #   (미지정 시 기본: 미션 시작 순간의 로봇 위치로 자동 복귀)
  #   ※ 기둥·장애물과 1m 이상 떨어진 빈 곳으로 지정할 것
  # 기타: -p min_targets:=3 -p approach_offset:=0.6 -p max_retries:=1

담당: 여수환 · 스마트해운물류 x ICT 멘토링 (파트 3) · Phase 4
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import String, Empty
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

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

        self.declare_parameter("min_targets", 3)        # 미션 시작에 필요한 확정 타겟 수
        self.declare_parameter("approach_offset", 0.6)  # [m] 타겟 중심 앞 정지 거리
        self.declare_parameter("max_retries", 1)        # 타겟당 재시도 횟수
        self.declare_parameter("return_home", True)     # [v2] 완료 후 복귀
        self.declare_parameter("wait_survey_done", True)  # [v4] 드론 탐사 완료 대기
        self.declare_parameter("use_home_pose", False)  # [v3] true 면 home_pose 좌표 사용
        self.declare_parameter("home_pose", [0.0, 0.0])  # [v3] 복귀 좌표 [x, y]
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("robot_frame", "base_link")

        g = lambda n: self.get_parameter(n).value
        self.min_targets = int(g("min_targets"))
        self.approach_offset = float(g("approach_offset"))
        self.max_retries = int(g("max_retries"))
        self.return_home = bool(g("return_home"))
        self.wait_survey_done = bool(g("wait_survey_done"))
        self.survey_done = False   # [v4] 🚁 드론 DONE 수신 여부
        self._await_new_survey = False  # [v5] 리셋 후 새 탐사 시작 대기 플래그
        self.use_home_pose = bool(g("use_home_pose"))
        hp = g("home_pose")
        self.home_pose_param = (float(hp[0]), float(hp[1]))
        self.world_frame = str(g("world_frame"))
        self.robot_frame = str(g("robot_frame"))

        if not TF2_AVAILABLE:
            raise RuntimeError("tf2_ros 가 필요합니다 (로봇 현재 위치 조회용).")
        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, self)  # noqa: F841

        self.create_subscription(PoseArray, "/targets/confirmed",
                                 self.on_targets, 10)
        # [v4] 🚁 드론 탐사 상태 구독 — "DONE" 이 오면 미션 시작 허용
        self.create_subscription(String, "/drone/survey_state",
                                 self.on_survey_state, 10)
        # [v5] 리셋 구독 — 시뮬 재시작 없이 재실행용
        self.create_subscription(Empty, "/mission/reset", self.on_reset, 10)
        # [우회] 전역 costmap 을 구독해 접근점이 실제로 비어 있는지 확인
        self.costmap = None
        self.declare_parameter("max_goal_cost", 40)
        self.max_goal_cost = int(self.get_parameter("max_goal_cost").value)
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._on_costmap,
            QoSProfile(depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── 미션 상태 ──
        self.targets = []          # [(x, y), ...] 확정 타겟 (미션 시작 시 고정)
        self.visited = []
        self.retries = []
        self.state = "WAITING"     # WAITING → NAVIGATE(반복) → RETURN → DONE
        self.current_idx = None
        self.home = None           # [v2] 미션 시작 시 로봇 위치
        self.home_retries = 0
        self.mission_start = None
        self._goal_handle = None

        self.timer = self.create_timer(1.0, self.on_timer)
        self.get_logger().info(
            f"mission_manager 시작 | 시작 조건: 확정 타겟 {self.min_targets}개, "
            f"접근 오프셋 {self.approach_offset}m, 재시도 {self.max_retries}회, "
            f"복귀 {'ON' if self.return_home else 'OFF'}, "
            f"탐사 완료 대기 {'ON' if self.wait_survey_done else 'OFF'}")

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
    def on_survey_state(self, msg: String):
        """[v4] 🚁 드론 탐사 상태. DONE 최초 수신 시 미션 시작을 허용한다.
        [v5] 리셋 직후에는 이전 탐사의 잔여 DONE 이 남아 있을 수 있으므로,
             DONE 이 아닌 상태(새 탐사의 MOVING 등)를 한 번 본 뒤의
             DONE 만 새 탐사 완료로 인정한다."""
        if self._await_new_survey:
            if msg.data != "DONE":
                self._await_new_survey = False   # 새 탐사가 시작됐다
            return
        if msg.data == "DONE" and not self.survey_done:
            self.survey_done = True
            self.get_logger().info("🚁 드론 탐사 완료 수신 — 로봇 미션 시작 준비")

    # ─────────────────────────────────────────────
    def on_reset(self, msg: Empty):
        """[v5] 미션 상태 초기화 — 다시 WAITING 부터.
        run_scenario.sh 가 ros2 param set 으로 갱신한 복귀 좌표를
        여기서 다시 읽는다 (시작 시 캐시한 값 갱신)."""
        g = lambda n: self.get_parameter(n).value
        self.use_home_pose = bool(g("use_home_pose"))
        hp = g("home_pose")
        self.home_pose_param = (float(hp[0]), float(hp[1]))
        self.targets = []
        self.visited = []
        self.retries = []
        self.current_idx = None
        self.home = None
        self.home_retries = 0
        self.survey_done = False
        self._await_new_survey = True   # 이전 탐사의 잔여 DONE 무시
        self.state = "WAITING"
        self.get_logger().info(
            f"◇ 미션 리셋 — 새 탐사 완료 대기 | 복귀 좌표 "
            f"({self.home_pose_param[0]:+.2f}, {self.home_pose_param[1]:+.2f})"
            f" [{'지정' if self.use_home_pose else '자동'}]")

    # ─────────────────────────────────────────────
    def on_targets(self, msg: PoseArray):
        """확정 타겟 리스트 갱신. 미션 시작 전에만 반영 (도중 변경 방지)."""
        if self.state != "WAITING":
            return
        self.targets = [(p.position.x, p.position.y) for p in msg.poses]

    # ─────────────────────────────────────────────
    def on_timer(self):
        if self.state == "WAITING":
            n = len(self.targets)
            # [v4] 시작 조건 분기
            if self.wait_survey_done:
                if not self.survey_done:
                    self.get_logger().info(
                        f"[WAITING] 🚁 드론 탐사 진행 중 — 현재 확정 타겟 {n}개",
                        throttle_duration_sec=5.0)
                    return
                if n < 1:
                    self.get_logger().warn(
                        "[WAITING] 탐사는 끝났으나 확정 타겟 0개 — 대기 "
                        "(탐사 범위에 목표가 없었을 수 있음)",
                        throttle_duration_sec=10.0)
                    return
                ready = True
            else:
                self.get_logger().info(
                    f"[WAITING] 확정 타겟 {n}/{self.min_targets} 대기 중",
                    throttle_duration_sec=5.0)
                ready = (n >= self.min_targets)
            if ready:
                # [v2] 미션 시작 순간의 로봇 위치를 홈으로 기록
                robot = self._robot_xy()
                if robot is None:
                    self.get_logger().warn("로봇 TF 대기 중 — 홈 기록 후 시작 예정",
                                           throttle_duration_sec=5.0)
                    return
                # [v3] 지정 좌표가 있으면 그것을, 없으면 시작 위치를 홈으로
                self.home = self.home_pose_param if self.use_home_pose else robot
                self.visited = [False] * len(self.targets)
                self.retries = [0] * len(self.targets)
                self.mission_start = self.get_clock().now()
                tlist = ", ".join(f"({x:+.2f},{y:+.2f})" for x, y in self.targets)
                self.get_logger().info(
                    f"■ 미션 시작 — 타겟 {len(self.targets)}개 고정: {tlist} | "
                    f"홈 ({self.home[0]:+.2f},{self.home[1]:+.2f}) "
                    f"[{'지정' if self.use_home_pose else '자동(시작위치)'}]")
                self.state = "SELECT"

        elif self.state == "SELECT":
            self._select_and_go()

        # NAVIGATE / RETURN 중에는 액션 콜백이 상태를 넘김. DONE 은 대기.

    # ─────────────────────────────────────────────
    def _make_goal(self, gx, gy, yaw):
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
        return goal

    def _send_goal(self, goal, done_cb):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 액션 서버 없음 — 1초 후 재시도")
            return False
        send = self.nav_client.send_goal_async(goal)
        send.add_done_callback(done_cb)
        return True

    # ─────────────────────────────────────────────
    def _select_and_go(self):
        remaining = [i for i, v in enumerate(self.visited) if not v]
        if not remaining:
            # 전 타겟 방문 완료 → 복귀 또는 종료
            if self.return_home and self.home is not None:
                self._go_home()
            else:
                self._finish()
            return

        robot = self._robot_xy()
        if robot is None:
            self.get_logger().warn("로봇 TF 조회 실패 — 1초 후 재시도",
                                   throttle_duration_sec=5.0)
            return
        rx, ry = robot

        # greedy: 현재 위치에서 가장 가까운 미방문 타겟
        idx = min(remaining,
                  key=lambda i: math.hypot(self.targets[i][0] - rx,
                                           self.targets[i][1] - ry))
        tx, ty = self.targets[idx]
        self.current_idx = idx

        # 접근 지점: 타겟 주위에서 costmap 상 비어 있는 곳을 고른다
        d = math.hypot(tx - rx, ty - ry)
        picked = self._pick_approach(tx, ty, rx, ry) if d > 1e-6 else None
        if picked is not None:
            gx, gy = picked
        elif d > 1e-6:
            gx = tx - (tx - rx) / d * self.approach_offset
            gy = ty - (ty - ry) / d * self.approach_offset
        else:
            gx, gy = tx, ty
        yaw = math.atan2(ty - gy, tx - gx)   # 타겟을 바라보는 방향

        self.get_logger().info(
            f"▶ 타겟 #{idx + 1} ({tx:+.2f},{ty:+.2f}) 로 출발 — "
            f"goal ({gx:+.2f},{gy:+.2f}), 로봇에서 {d:.2f}m")
        if self._send_goal(self._make_goal(gx, gy, yaw), self._on_goal_response):
            self.state = "NAVIGATE"

    # ─────────────────────────────────────────────
    def _on_costmap(self, msg):
        self.costmap = msg

    def _cost_at(self, x, y):
        """map 좌표의 costmap 값. 범위 밖이거나 미수신이면 None."""
        cm = self.costmap
        if cm is None:
            return None
        res = cm.info.resolution
        col = int((x - cm.info.origin.position.x) / res)
        row = int((y - cm.info.origin.position.y) / res)
        if not (0 <= col < cm.info.width and 0 <= row < cm.info.height):
            return None
        return cm.data[row * cm.info.width + col]

    def _pick_approach(self, tx, ty, rx, ry):
        """타겟 주위를 둘러 비어 있는 접근점 중 로봇에서 가장 가까운 곳."""
        if self.costmap is None:
            return None
        base = math.atan2(ry - ty, rx - tx)
        for extra in (0.0, 0.25, 0.50):
            r = self.approach_offset + extra
            best = None
            unknown = None
            for k in range(24):
                step = ((k + 1) // 2) * (math.pi / 12.0)
                ang = base + (step if k % 2 == 0 else -step)
                gx = tx + r * math.cos(ang)
                gy = ty + r * math.sin(ang)
                c = self._cost_at(gx, gy)
                if c is None:
                    continue
                dd = math.hypot(gx - rx, gy - ry)
                if 0 <= c <= self.max_goal_cost:
                    if best is None or dd < best[0]:
                        best = (dd, gx, gy)
                elif c < 0:
                    # 미탐색 = 아직 안 본 곳. 막힌 것이 아니므로 차선책으로 둔다
                    if unknown is None or dd < unknown[0]:
                        unknown = (dd, gx, gy)
            pick, kind = (best, "탐색됨") if best else (unknown, "미탐색")
            if pick is not None:
                self.get_logger().info(
                    f"  접근점 탐색: 반경 {r:.2f}m {kind} "
                    f"({pick[1]:+.2f},{pick[2]:+.2f})")
                return pick[1], pick[2]
        self.get_logger().warn("  접근점 탐색 실패 — 직선 방식으로 대체")
        return None

    def _go_home(self):
        """[v2] 홈으로 복귀."""
        hx, hy = self.home
        robot = self._robot_xy()
        if robot is None:
            self.get_logger().warn("로봇 TF 조회 실패 — 1초 후 재시도",
                                   throttle_duration_sec=5.0)
            return
        rx, ry = robot
        d = math.hypot(hx - rx, hy - ry)
        yaw = math.atan2(hy - ry, hx - rx) if d > 1e-6 else 0.0

        self.get_logger().info(
            f"◀ 전 타겟 방문 완료 — 홈 ({hx:+.2f},{hy:+.2f}) 으로 복귀 ({d:.2f}m)")
        if self._send_goal(self._make_goal(hx, hy, yaw), self._on_home_response):
            self.state = "RETURN"

    # ─────────────────────────────────────────────
    def _finish(self):
        dt = (self.get_clock().now() - self.mission_start).nanoseconds * 1e-9
        ok = sum(1 for i, v in enumerate(self.visited)
                 if v and self.retries[i] <= self.max_retries)
        home_str = " + 홈 복귀" if (self.return_home and self.state == "RETURN") else ""
        self.get_logger().info(
            f"■■ 미션 완료 — {len(self.targets)}개 중 방문 처리 {ok}개{home_str}, "
            f"소요 {dt:.1f}초")
        self.state = "DONE"

    # ─────────────────────────────────────────────
    # 타겟 주행 콜백
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
            self.visited[idx] = True   # 포기 처리 → 순회에서 제외
            self.get_logger().warn(
                f"타겟 #{idx + 1} 재시도 초과 — 포기하고 다음 타겟으로")
        self.state = "SELECT"

    # ─────────────────────────────────────────────
    # 홈 복귀 콜백
    def _on_home_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("홈 복귀 goal 거부됨 — 재시도")
            self._home_failure()
            return
        handle.get_result_async().add_done_callback(self._on_home_result)

    def _on_home_result(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info("✔ 홈 도착")
            self._finish()
        else:
            self.get_logger().warn(f"✘ 홈 복귀 실패 (status={status})")
            self._home_failure()

    def _home_failure(self):
        self.home_retries += 1
        if self.home_retries > self.max_retries:
            self.get_logger().warn("홈 복귀 재시도 초과 — 현 위치에서 미션 종료")
            self._finish()
        else:
            self.state = "SELECT"   # SELECT 가 remaining 없음을 보고 _go_home 재호출


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
