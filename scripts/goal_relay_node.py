"""
goal_relay_node.py — 파트2·3 연동 다리 (변환 노드)
====================================================
파트2(obstacle_locator_node)가 발행하는 "목표의 map 절대좌표"를
파트3 Nav2가 이해하는 "목표 자세(/goal_pose)"로 바꿔주는 노드.

  파트2 /obstacle/ground_point (geometry_msgs/Point, map 기준 x,y)
       │  ↓  [이 노드]  ← standoff 만큼 앞에서 멈추도록 목표 보정
  파트3 /goal_pose (geometry_msgs/PoseStamped, map 프레임)
       │  ↓
  Nav2 → 장애물 피해 목표 앞까지 자율주행

핵심 설계:
  1. 목표가 일정 거리(move_threshold) 이상 바뀌었을 때만 한 번 발행
     (obstacle_locator는 2Hz로 계속 뱉으므로, 그대로 흘리면 Nav2가 리셋 반복).
  2. standoff: 목표(기둥)는 시각 전용 마커라 그 위치로 그대로 가면 로봇이
     겹쳐버림. 그래서 "로봇→목표 방향"으로 standoff[m] 앞 지점을 목표로 삼아
     기둥 앞에서 멈추고 기둥을 바라보게 한다. (로봇 위치는 TF map→robot_frame 조회)

파라미터:
  - input_topic    (기본 /obstacle/ground_point)
  - goal_topic     (기본 /goal_pose)
  - goal_frame     (기본 map)
  - robot_frame    (기본 base_link)   : standoff 방향 계산용 로봇 프레임
  - move_threshold (기본 0.15) [m]     : 이만큼 바뀌어야 새 목표로 인정
  - standoff       (기본 0.5) [m]      : 목표에서 이만큼 앞에 멈춤 (0이면 정확히 목표점)

사용 예:
  python3 goal_relay_node.py
  python3 goal_relay_node.py --ros-args -p standoff:=0.6 -p robot_frame:=base_link

담당: 파트2·3 통합 · 프로젝트: 스마트해운물류 x ICT 멘토링 (갯끈풀 탐지)
"""

import math

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Point, PoseStamped

from tf2_ros import Buffer, TransformListener


class GoalRelayNode(Node):
    def __init__(self):
        super().__init__("goal_relay_node")

        # ── 파라미터 ──
        self.declare_parameter("input_topic", "/obstacle/ground_point")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("move_threshold", 0.15)   # [m]
        self.declare_parameter("standoff", 0.8)           # [m] 기둥반경0.3+Nav2팽창0.55 고려

        self.goal_frame = self.get_parameter("goal_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.move_threshold = float(self.get_parameter("move_threshold").value)
        self.standoff = float(self.get_parameter("standoff").value)
        input_topic = self.get_parameter("input_topic").value
        goal_topic = self.get_parameter("goal_topic").value

        # 마지막으로 처리한 목표 원점 (중복 발행 방지용)
        self.last_target = None  # (x, y)

        # ── TF: 로봇 현재 위치 조회 (standoff 방향 계산용) ──
        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, self)

        # ── 입력: 파트2의 map 절대좌표 ──
        self.create_subscription(Point, input_topic, self.on_ground_point, 10)

        # ── 출력: Nav2 목표. RViz "2D Goal Pose" 처럼 latched(TRANSIENT_LOCAL) ──
        goal_qos = QoSProfile(depth=1)
        goal_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        goal_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.pub_goal = self.create_publisher(PoseStamped, goal_topic, goal_qos)

        self.get_logger().info(
            f"goal_relay_node 시작 | 입력={input_topic} → 출력={goal_topic} "
            f"(frame={self.goal_frame}, 변화기준={self.move_threshold}m, standoff={self.standoff}m)"
        )

    def _robot_xy(self):
        """map 기준 로봇 현재 (x, y). 조회 실패 시 None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.goal_frame, self.robot_frame, rclpy.time.Time()
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return None

    def on_ground_point(self, msg: Point):
        tx, ty = float(msg.x), float(msg.y)

        # 목표 원점이 충분히 바뀌었을 때만 새 목표로 발행
        if self.last_target is not None:
            if math.hypot(tx - self.last_target[0], ty - self.last_target[1]) < self.move_threshold:
                return

        # ── standoff 적용: 로봇→목표 방향으로 standoff 앞 지점을 목표로 ──
        gx, gy = tx, ty
        yaw = 0.0
        robot = self._robot_xy()
        if robot is not None:
            rx, ry = robot
            dx, dy = tx - rx, ty - ry
            dist = math.hypot(dx, dy)
            yaw = math.atan2(dy, dx)  # 목표를 바라보는 방향
            if dist > 1e-3:
                # 목표에서 standoff 만큼 뒤로(로봇 쪽으로) 물러난 지점
                back = min(self.standoff, dist)  # 목표를 지나치지 않도록 clamp
                gx = tx - (dx / dist) * back
                gy = ty - (dy / dist) * back
        else:
            self.get_logger().warn(
                f"TF {self.goal_frame}→{self.robot_frame} 조회 실패 → standoff 없이 목표점 그대로 사용"
            )

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.goal_frame
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.pub_goal.publish(goal)
        self.last_target = (tx, ty)
        self.get_logger().info(
            f"새 목표 발행 → /goal_pose (목표 {tx:+.2f},{ty:+.2f} → "
            f"멈춤지점 {gx:+.2f},{gy:+.2f}, yaw={math.degrees(yaw):+.1f}°)"
        )


def main():
    rclpy.init()
    node = GoalRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
