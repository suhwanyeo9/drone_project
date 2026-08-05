import numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from collections import deque

TRUTH = {'target_spartina': (-0.75, 0.20),
         'target_spartina_2': (2.25, 2.05),
         'target_spartina_3': (1.85, -2.05)}
H_CAM, R_PILLAR = 8.0, 0.30
K = {}

def on_info(msg):
    K['fx'], K['cx'], K['cy'] = msg.k[0], msg.k[2], msg.k[5]

def cb(msg):
    if 'fx' not in K:
        return
    fx, cx, cy = K['fx'], K['cx'], K['cy']
    s = H_CAM / fx
    print(f"camera_info: fx={fx:.2f} cx={cx:.1f} cy={cy:.1f}  지면축척={s:.6f} m/px\n")

    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding == 'bgr8':
        a = a[:, :, ::-1]
    r, g, b = a[:,:,0].astype(int), a[:,:,1].astype(int), a[:,:,2].astype(int)
    m = (g > r*1.4) & (g > b*1.4) & (g > 40)

    lab = np.zeros(m.shape, int); n = 0
    for i in range(m.shape[0]):
        for j in range(m.shape[1]):
            if m[i,j] and lab[i,j] == 0:
                n += 1; q = deque([(i,j)]); lab[i,j] = n
                while q:
                    y,x = q.popleft()
                    for dy,dx in ((1,0),(-1,0),(0,1),(0,-1)):
                        ny,nx = y+dy, x+dx
                        if 0<=ny<m.shape[0] and 0<=nx<m.shape[1] and m[ny,nx] and lab[ny,nx]==0:
                            lab[ny,nx]=n; q.append((ny,nx))

    print(f'{"픽셀(u,v)":>15} {"무게중심 투영":>17} {"밑동 보정":>17}  정답 / 오차')
    for k in range(1, n+1):
        ys, xs = np.where(lab==k)
        if len(ys) < 30: continue
        u, v = xs.mean(), ys.mean()
        Xc, Yc = -(v-cy)*s, -(u-cx)*s                       # 무게중심 그대로

        d = np.hypot(xs-cx, ys-cy)                          # 중심에서 가장 먼 점 = 밑동 바깥날
        far = d.max() * s - R_PILLAR
        ang = np.arctan2(-(ys[d.argmax()]-cy), -(xs[d.argmax()]-cx))
        Xb, Yb = far*np.cos(ang), far*np.sin(ang)

        name, e1 = min(((t, np.hypot(Xc-a1, Yc-a2)) for t,(a1,a2) in TRUTH.items()), key=lambda z:z[1])
        e2 = np.hypot(Xb-TRUTH[name][0], Yb-TRUTH[name][1])
        print(f'({u:6.1f},{v:6.1f}) ({Xc:+6.2f},{Yc:+6.2f}) {e1:5.2f}m '
              f'({Xb:+6.2f},{Yb:+6.2f}) {e2:5.2f}m  {name}')
    rclpy.shutdown()

rclpy.init(); nd = Node('pillars')
nd.create_subscription(CameraInfo, '/drone/camera_info', on_info, 10)
nd.create_subscription(Image, '/drone/image_raw', cb, 1)
rclpy.spin(nd)
