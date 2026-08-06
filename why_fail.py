import math, rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

TX, TY = 1.85, -2.05
RX, RY = -2.0, 0.0
MAXC = 40

class Chk(Node):
    def __init__(self):
        super().__init__('why_fail')
        self.cm = None
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.on_cm,
            QoSProfile(depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(3.0, self.run)

    def on_cm(self, m):
        self.cm = m

    def cost(self, x, y):
        cm = self.cm
        if cm is None:
            return 'NO_MAP'
        r = cm.info.resolution
        c = int((x - cm.info.origin.position.x) / r)
        w = int((y - cm.info.origin.position.y) / r)
        if not (0 <= c < cm.info.width and 0 <= w < cm.info.height):
            return 'OUT'
        return cm.data[w * cm.info.width + c]

    def run(self):
        if self.cm is None:
            self.get_logger().warn('costmap 미수신 — 이것만으로 원인 확정')
            return
        i = self.cm.info
        self.get_logger().info(
            f'costmap {i.width}x{i.height} res {i.resolution} '
            f'origin ({i.origin.position.x:.2f},{i.origin.position.y:.2f})')
        base = math.atan2(RY - TY, RX - TX)
        for extra in (0.0, 0.25, 0.50):
            rad = 0.6 + extra
            out = []
            for k in range(24):
                step = ((k + 1) // 2) * (math.pi / 12.0)
                a = base + (step if k % 2 == 0 else -step)
                gx, gy = TX + rad*math.cos(a), TY + rad*math.sin(a)
                c = self.cost(gx, gy)
                ok = isinstance(c, int) and 0 <= c <= MAXC
                out.append(f'{math.degrees(a)%360:5.0f}deg({gx:+.2f},{gy:+.2f})='
                           f'{c}{"O" if ok else "X"}')
            self.get_logger().info(f'--- 반경 {rad:.2f} ---\n' + '\n'.join(out))
        rclpy.shutdown()

rclpy.init(); rclpy.spin(Chk())
