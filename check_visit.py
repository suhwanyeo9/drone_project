import math, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

TARGETS = {'#1': (2.25, 2.05), '#2': (1.85, -2.05), '#3': (-0.75, 0.20)}
HOME = (-3.0, -3.0)

class Watch(Node):
    def __init__(self):
        super().__init__('check_visit')
        self.best = {k: (9e9, 0.0, 0.0) for k in TARGETS}
        self.best_home = (9e9, 0.0, 0.0)
        self.n = 0
        self.create_subscription(Odometry, '/odom', self.cb, 10)
        self.create_timer(5.0, self.report)
        self.get_logger().info('기록 시작 — Ctrl+C 로 종료')

    def cb(self, m):
        x = m.pose.pose.position.x
        y = m.pose.pose.position.y
        self.n += 1
        for k, (tx, ty) in TARGETS.items():
            d = math.hypot(x - tx, y - ty)
            if d < self.best[k][0]:
                self.best[k] = (d, x, y)
        d = math.hypot(x - HOME[0], y - HOME[1])
        if d < self.best_home[0]:
            self.best_home = (d, x, y)

    def report(self):
        lines = [f'[{self.n}샘플] 타겟별 최근접 거리']
        for k in sorted(TARGETS):
            d, x, y = self.best[k]
            mark = 'O' if d < 1.5 else ('~' if d < 2.5 else 'X')
            lines.append(f'  {mark} {k} {TARGETS[k]} ← {d:.2f} m  (로봇 {x:+.2f},{y:+.2f})')
        d, x, y = self.best_home
        lines.append(f'    홈 {HOME} ← {d:.2f} m')
        self.get_logger().info('\n'.join(lines))

rclpy.init(); rclpy.spin(Watch())
