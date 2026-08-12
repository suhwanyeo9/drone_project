#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arena01 SLAM 측정값 -> Gazebo world + Nav2 지도(pgm/yaml) 동시 생성

측정 원본과 무관하게 아래 파라미터만으로 완전한 직사각형 방을 만든다.
(SLAM의 회전 1.4deg / 스큐 2.4deg는 여기서 자동으로 제거됨)

사용법:
    python3 make_arena01.py
필요: numpy 만
"""
import os
import re
import sys
import numpy as np

# ============================================================
# 파라미터 - 여기만 고치면 됨
# ============================================================
SCALE      = 2.0     # 길이 배율 (넓이 = SCALE^2 = 4배)
ROOM_W     = 3.05    # 실물 방 가로 x (m) - arena01 측정값
ROOM_H     = 3.00    # 실물 방 세로 y (m) - arena01 측정값

WALL_T     = 0.10    # 벽 두께 (m)
WALL_H     = 1.00    # 벽 높이 (m)
OBST_H     = 0.50    # 장애물 높이 (m)

RES        = 0.05    # Nav2 지도 해상도 (m/px) - 기존과 동일하게 유지

# 실물 스케일 장애물: (x, y, 가로, 세로) 방 중심 기준 (m)
OBSTACLES = [
    (-0.58,  0.81, 0.30, 0.30),
    ( 0.59,  0.77, 0.40, 0.35),
    (-0.65, -0.40, 0.35, 0.40),
    ( 0.60, -0.73, 0.25, 0.35),
]

# 갯끈풀 목표 기둥: (이름, x, y) - 확대 후 좌표계 기준 (m)
TARGETS = [
    ('target_spartina',   -2.30,  2.30),
    ('target_spartina_2',  2.30,  2.30),
    ('target_spartina_3', -2.30, -2.30),
]
TARGET_R = 0.15
TARGET_H = 1.00
TARGET_IN_MAP = False   # True면 목표 기둥도 Nav2 지도에 장애물로 표시

SPAWN = (-2.0, 0.0)     # 로봇 스폰 (launch 기본값과 일치해야 함)

OUT_DIR    = os.path.expanduser('~/drone_project')
WORLD_PATH = os.path.join(OUT_DIR, 'worlds', 'arena01.world')
MAP_STEM   = os.path.join(OUT_DIR, 'maps', 'arena01_map')
OLD_WORLD  = os.path.join(OUT_DIR, 'worlds', 'slam_room.world')  # 카메라 블록 재사용용

# ============================================================
# 유도 값
# ============================================================
W = ROOM_W * SCALE          # 방 내부 가로
H = ROOM_H * SCALE          # 방 내부 세로
HX, HY = W / 2.0, H / 2.0   # 내부 반폭
OX, OY = HX + WALL_T, HY + WALL_T   # 벽 바깥까지 반폭

# ============================================================
# 드론 카메라 - 기존 world에서 그대로 복사 (없으면 기본 템플릿)
# ============================================================
CAM_FALLBACK = """    <model name="drone_camera">
      <static>true</static>
      <pose>0 0 8 0 1.5708 0</pose>
      <link name="link">
        <sensor name="drone_cam" type="camera">
          <camera>
            <horizontal_fov>1.047</horizontal_fov>
            <image><width>640</width><height>480</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>100</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
          <visualize>false</visualize>
          <plugin name="drone_camera_controller" filename="libgazebo_ros_camera.so">
            <ros>
              <namespace>/drone</namespace>
              <remapping>~/image_raw:=image_raw</remapping>
              <remapping>~/camera_info:=camera_info</remapping>
            </ros>
            <frame_name>drone_camera_optical</frame_name>
          </plugin>
        </sensor>
      </link>
    </model>
"""


def extract_camera(path):
    """기존 world에서 카메라 모델 블록을 통째로 꺼내온다."""
    if not os.path.isfile(path):
        return None
    txt = open(path, encoding='utf-8').read()
    m = re.search(r'[ \t]*<model name="drone_camera">', txt)
    if not m:
        return None
    start = m.start()
    depth, i = 0, m.start()
    for tag in re.finditer(r'<model\b|</model>', txt[start:]):
        if tag.group().startswith('<model'):
            depth += 1
        else:
            depth -= 1
            if depth == 0:
                i = start + tag.end()
                return txt[start:i] + '\n'
    return None


# ============================================================
# world 생성
# ============================================================
def box(name, x, y, z, sx, sy, sz, rgba):
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="c">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material>
            <ambient>{rgba}</ambient>
            <diffuse>{rgba}</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


def cyl(name, x, y, r, h, rgba):
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {h/2:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="c">
          <geometry><cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>
        </collision>
        <visual name="v">
          <geometry><cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>
          <material>
            <ambient>{rgba}</ambient>
            <diffuse>{rgba}</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


def build_world(cam_block):
    GREY = '0.7 0.7 0.7 1'
    BLUE = '0.4 0.45 0.6 1'
    GREEN = '0.8 0.1 0.1 1'
    parts = []

    # 벽 4장 (모서리까지 덮도록 상하벽을 좌우로 늘림)
    parts.append(box('wall_north', 0,  HY + WALL_T/2, WALL_H/2, W + 2*WALL_T, WALL_T, WALL_H, GREY))
    parts.append(box('wall_south', 0, -HY - WALL_T/2, WALL_H/2, W + 2*WALL_T, WALL_T, WALL_H, GREY))
    parts.append(box('wall_east',  HX + WALL_T/2, 0, WALL_H/2, WALL_T, H, WALL_H, GREY))
    parts.append(box('wall_west', -HX - WALL_T/2, 0, WALL_H/2, WALL_T, H, WALL_H, GREY))

    # 장애물
    for i, (x, y, w, h) in enumerate(OBSTACLES, 1):
        parts.append(box(f'obstacle_{i}', x*SCALE, y*SCALE, OBST_H/2,
                         w*SCALE, h*SCALE, OBST_H, BLUE))

    # 목표 기둥
    for name, x, y in TARGETS:
        parts.append(cyl(name, x, y, TARGET_R, TARGET_H, GREEN))

    body = ''.join(parts)
    cam = cam_block if cam_block else CAM_FALLBACK

    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="arena01">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <shadows>false</shadows>
    </scene>

{body}
{cam}
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>50.0</update_rate>
    </plugin>

  </world>
</sdf>
"""


# ============================================================
# Nav2 지도 생성 (world와 같은 파라미터에서 직접 생성 = 완전 일치)
# ============================================================
def build_map():
    w_px = int(round(2*OX / RES))
    h_px = int(round(2*OY / RES))
    grid = np.full((h_px, w_px), 254, dtype=np.uint8)   # 254 = free
    ox, oy = -OX, -OY

    def fill(x0, x1, y0, y1, val):
        c0 = int(np.floor((x0 - ox) / RES))
        c1 = int(np.ceil((x1 - ox) / RES))
        r1 = h_px - int(np.floor((y0 - oy) / RES))
        r0 = h_px - int(np.ceil((y1 - oy) / RES))
        c0, c1 = max(0, c0), min(w_px, c1)
        r0, r1 = max(0, r0), min(h_px, r1)
        if c1 > c0 and r1 > r0:
            grid[r0:r1, c0:c1] = val

    # 벽
    fill(-OX, OX,  HY, OY, 0)
    fill(-OX, OX, -OY, -HY, 0)
    fill( HX, OX, -OY, OY, 0)
    fill(-OX, -HX, -OY, OY, 0)

    # 장애물
    for x, y, w, h in OBSTACLES:
        X, Y, WW, HH = x*SCALE, y*SCALE, w*SCALE, h*SCALE
        fill(X - WW/2, X + WW/2, Y - HH/2, Y + HH/2, 0)

    if TARGET_IN_MAP:
        for _, x, y in TARGETS:
            fill(x - TARGET_R, x + TARGET_R, y - TARGET_R, y + TARGET_R, 0)

    return grid, (ox, oy)


def write_pgm(path, grid):
    h, w = grid.shape
    with open(path, 'wb') as f:
        f.write(b'P5\n')
        f.write(f'{w} {h}\n255\n'.encode())
        f.write(grid.tobytes())


def write_yaml(path, pgm_name, origin):
    with open(path, 'w', encoding='utf-8') as f:
        f.write(f"""image: {pgm_name}
mode: trinary
resolution: {RES}
origin: [{origin[0]:.3f}, {origin[1]:.3f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
""")


# ============================================================
# 검증
# ============================================================
def check():
    warn = []
    rects = []
    for i, (x, y, w, h) in enumerate(OBSTACLES, 1):
        rects.append((f'obstacle_{i}', x*SCALE, y*SCALE, w*SCALE, h*SCALE))

    # 목표 기둥 vs 장애물 겹침
    for name, tx, ty in TARGETS:
        for rn, rx, ry, rw, rh in rects:
            if (abs(tx - rx) < rw/2 + TARGET_R) and (abs(ty - ry) < rh/2 + TARGET_R):
                warn.append(f'  [겹침] {name} <-> {rn}')
        if abs(tx) + TARGET_R > HX or abs(ty) + TARGET_R > HY:
            warn.append(f'  [방 밖] {name}')

    # 스폰 지점 여유
    sx, sy = SPAWN
    clear = min(HX - abs(sx), HY - abs(sy))
    for rn, rx, ry, rw, rh in rects:
        d = max(abs(sx - rx) - rw/2, abs(sy - ry) - rh/2)
        clear = min(clear, d)
    for name, tx, ty in TARGETS:
        d = np.hypot(sx - tx, sy - ty) - TARGET_R
        clear = min(clear, d)
    if clear < 0.30:
        warn.append(f'  [스폰 협소] 최소 여유 {clear:.2f} m (로봇 반경 0.22 m)')

    # 드론 카메라 화각 (8m, HFOV 1.047, 640x480)
    fov_y = 2 * 8 * np.tan(1.047/2)
    fov_x = 2 * 8 * np.tan(np.arctan(np.tan(1.047/2) * 480/640))
    if W > fov_x or H > fov_y:
        warn.append(f'  [카메라 화각 부족] 촬영 {fov_x:.2f} x {fov_y:.2f} m < 방 {W:.2f} x {H:.2f} m')

    return warn, clear, (fov_x, fov_y)


# ============================================================
def main():
    os.makedirs(os.path.join(OUT_DIR, 'worlds'), exist_ok=True)
    os.makedirs(os.path.join(OUT_DIR, 'maps'), exist_ok=True)

    cam = extract_camera(OLD_WORLD)
    print('드론 카메라: ' + ('기존 slam_room.world에서 복사' if cam else '기본 템플릿 사용 (원본 못 찾음)'))

    world = build_world(cam)
    with open(WORLD_PATH, 'w', encoding='utf-8') as f:
        f.write(world)

    grid, origin = build_map()
    write_pgm(MAP_STEM + '.pgm', grid)
    write_yaml(MAP_STEM + '.yaml', os.path.basename(MAP_STEM) + '.pgm', origin)

    warn, clear, fov = check()

    print()
    print('=' * 52)
    print(f'방 내부      {W:.2f} x {H:.2f} m  (실물 {ROOM_W} x {ROOM_H}, {SCALE}배)')
    print(f'벽 포함      {2*OX:.2f} x {2*OY:.2f} m')
    print(f'지도         {grid.shape[1]} x {grid.shape[0]} px @ {RES} m/px')
    print(f'지도 origin  [{origin[0]:.3f}, {origin[1]:.3f}, 0.0]')
    print(f'모델 수      벽 4 + 장애물 {len(OBSTACLES)} + 목표 {len(TARGETS)} + 카메라 1')
    print(f'스폰 여유    {clear:.2f} m  @ ({SPAWN[0]}, {SPAWN[1]})')
    print(f'카메라 촬영  {fov[0]:.2f} x {fov[1]:.2f} m')
    print('=' * 52)
    print(f'world  -> {WORLD_PATH}')
    print(f'map    -> {MAP_STEM}.pgm / .yaml')

    if warn:
        print('\n경고:')
        print('\n'.join(warn))
    else:
        print('\n검증 통과 - 겹침/여유/화각 모두 이상 없음')

    print(f'\n[다음] launch 파일의 world 경로를 arena01.world 로 교체')
    print(f'       T4 기동 로그에서 확인할 값: '
          f'"Resizing costmap to {grid.shape[1]} X {grid.shape[0]}"')


if __name__ == '__main__':
    main()
