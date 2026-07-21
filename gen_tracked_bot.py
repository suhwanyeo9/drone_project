#!/usr/bin/env python3
"""
gen_tracked_bot.py — 무한궤도 차량 URDF 생성기

사용법:
    python3 gen_tracked_bot.py 1.5

인자는 배율(k). k=1.0 이 기준 크기(전장 0.35m).
    k=1.0  ->  0.350 x 0.250 x 0.100   (현재)
    k=1.5  ->  0.525 x 0.375 x 0.150
    k=2.0  ->  0.700 x 0.500 x 0.200   (처음 만들었던 크기)

URDF 를 ~/drone_project/urdf/tracked_bot.urdf 에 쓰고,
같이 맞춰야 할 Nav2 설정값을 화면에 출력한다.
"""

import sys
import os

# ---------------- 기준 치수 (k = 1.0) ----------------
BODY_L, BODY_W, BODY_H = 0.35, 0.25, 0.10   # 차체
WHEEL_R, WHEEL_T = 0.05, 0.04               # 구동륜 반지름 / 폭
WHEEL_X, WHEEL_Y = 0.11, 0.125              # 구동륜 앞뒤 / 좌우 오프셋
LIDAR_X, LIDAR_Z = 0.075, 0.10              # 라이다 위치 (base_link 기준)
BODY_MASS, WHEEL_MASS = 2.0, 0.15           # 질량
BASE_SPEED = 0.3                            # 최고 속도 m/s
FOOT_L, FOOT_W = 0.19, 0.14                 # Nav2 footprint 반치수
INFLATION = 0.35                            # Nav2 inflation_radius


def box_inertia(m, x, y, z):
    return (m / 12.0 * (y * y + z * z),
            m / 12.0 * (x * x + z * z),
            m / 12.0 * (x * x + y * y))


def cyl_inertia(m, r, h):
    ixx = m / 12.0 * (3 * r * r + h * h)
    return (ixx, ixx, 0.5 * m * r * r)


def build(k):
    # 길이는 k배, 질량은 k^3배 (부피 비례), 토크는 k^4배 (질량 x 반지름)
    L, W, H = BODY_L * k, BODY_W * k, BODY_H * k
    r, t = WHEEL_R * k, WHEEL_T * k
    wx, wy = WHEEL_X * k, WHEEL_Y * k
    lx, lz = LIDAR_X * k, LIDAR_Z * k

    clearance = r                 # 지상고 = 바퀴 반지름
    base_z = clearance + H / 2.0  # base_footprint -> base_link
    wheel_z = -H / 2.0            # base_link 중심에서 바퀴 중심까지

    m_body = BODY_MASS * k ** 3
    m_wheel = WHEEL_MASS * k ** 3
    bxx, byy, bzz = box_inertia(m_body, L, W, H)
    wxx, wyy, wzz = cyl_inertia(m_wheel, r, t)

    sep, dia = 2 * wy, 2 * r
    torque = max(10.0, 10.0 * k ** 4)
    speed = BASE_SPEED * k
    scan_min = 0.12 * k

    def wheel(name, sx, sy):
        return f'''
  <joint name="wheel_{name}_joint" type="continuous">
    <parent link="base_link"/>
    <child  link="wheel_{name}_link"/>
    <origin xyz="{sx:.4f} {sy:.4f} {wheel_z:.4f}" rpy="-1.570796 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <link name="wheel_{name}_link">
    <visual>
      <geometry><cylinder length="{t:.4f}" radius="{r:.4f}"/></geometry>
      <material name="wheel_dark"/>
    </visual>
    <collision>
      <geometry><cylinder length="{t:.4f}" radius="{r:.4f}"/></geometry>
    </collision>
    <inertial>
      <mass value="{m_wheel:.5f}"/>
      <inertia ixx="{wxx:.8f}" ixy="0.0" ixz="0.0"
               iyy="{wyy:.8f}" iyz="0.0" izz="{wzz:.8f}"/>
    </inertial>
  </link>'''

    def friction(name):
        return f'''
  <gazebo reference="wheel_{name}_link">
    <mu1>1.0</mu1><mu2>0.3</mu2>
    <fdir1>1 0 0</fdir1>
    <kp>{100000.0 * k:.1f}</kp><kd>{5.0 * k:.1f}</kd>
    <minDepth>0.001</minDepth>
    <material>Gazebo/Black</material>
  </gazebo>'''

    def track(side, sy):
        return f'''
  <joint name="track_{side}_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="track_{side}_link"/>
    <origin xyz="0 {sy:.4f} {wheel_z:.4f}" rpy="0 0 0"/>
  </joint>
  <link name="track_{side}_link">
    <visual>
      <geometry><box size="{L * 1.03:.4f} {t * 1.25:.4f} {2 * r * 1.1:.4f}"/></geometry>
      <material name="track_black"/>
    </visual>
    <inertial>
      <mass value="0.005"/>
      <inertia ixx="1e-7" ixy="0.0" ixz="0.0" iyy="1e-7" iyz="0.0" izz="1e-7"/>
    </inertial>
  </link>'''

    urdf = f'''<?xml version="1.0"?>
<!-- gen_tracked_bot.py 로 생성  |  배율 k = {k}
     차체 {L:.3f} x {W:.3f} x {H:.3f} m  |  질량 {m_body:.2f} kg
     트랙간격 {sep:.3f} m  |  구동륜지름 {dia:.3f} m  |  라이다 지상 {base_z + lz:.3f} m -->
<robot name="tracked_bot">

  <material name="body_gray">  <color rgba="0.30 0.32 0.35 1.0"/></material>
  <material name="track_black"><color rgba="0.10 0.10 0.10 1.0"/></material>
  <material name="wheel_dark"> <color rgba="0.05 0.05 0.05 1.0"/></material>
  <material name="sensor_blue"><color rgba="0.10 0.30 0.70 1.0"/></material>

  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child  link="base_link"/>
    <origin xyz="0 0 {base_z:.4f}" rpy="0 0 0"/>
  </joint>

  <link name="base_link">
    <visual>
      <geometry><box size="{L:.4f} {W:.4f} {H:.4f}"/></geometry>
      <material name="body_gray"/>
    </visual>
    <collision>
      <geometry><box size="{L:.4f} {W:.4f} {H:.4f}"/></geometry>
    </collision>
    <inertial>
      <mass value="{m_body:.4f}"/>
      <inertia ixx="{bxx:.6f}" ixy="0.0" ixz="0.0"
               iyy="{byy:.6f}" iyz="0.0" izz="{bzz:.6f}"/>
    </inertial>
  </link>
{track("left", wy)}
{track("right", -wy)}
{wheel("left_front", wx, wy)}
{wheel("left_rear", -wx, wy)}
{wheel("right_front", wx, -wy)}
{wheel("right_rear", -wx, -wy)}

  <joint name="scan_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="base_scan"/>
    <origin xyz="{lx:.4f} 0 {lz:.4f}" rpy="0 0 0"/>
  </joint>
  <link name="base_scan">
    <visual>
      <geometry><cylinder length="{0.025 * k:.4f}" radius="{0.02 * k:.4f}"/></geometry>
      <material name="sensor_blue"/>
    </visual>
    <collision>
      <geometry><cylinder length="{0.025 * k:.4f}" radius="{0.02 * k:.4f}"/></geometry>
    </collision>
    <inertial>
      <mass value="{0.05 * k ** 3:.5f}"/>
      <inertia ixx="1e-5" ixy="0.0" ixz="0.0" iyy="1e-5" iyz="0.0" izz="1e-5"/>
    </inertial>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="imu_link"/>
    <origin xyz="0 0 {H / 2:.4f}" rpy="0 0 0"/>
  </joint>
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="1e-6" ixy="0.0" ixz="0.0" iyy="1e-6" iyz="0.0" izz="1e-6"/>
    </inertial>
  </link>

  <gazebo reference="base_link"><material>Gazebo/DarkGrey</material></gazebo>
  <gazebo reference="track_left_link"><material>Gazebo/Black</material></gazebo>
  <gazebo reference="track_right_link"><material>Gazebo/Black</material></gazebo>
{friction("left_front")}
{friction("left_rear")}
{friction("right_front")}
{friction("right_rear")}

  <gazebo>
    <plugin name="tracked_diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <remapping>cmd_vel:=cmd_vel</remapping>
        <remapping>odom:=odom</remapping>
      </ros>
      <update_rate>30</update_rate>
      <num_wheel_pairs>2</num_wheel_pairs>
      <left_joint>wheel_left_front_joint</left_joint>
      <left_joint>wheel_left_rear_joint</left_joint>
      <right_joint>wheel_right_front_joint</right_joint>
      <right_joint>wheel_right_rear_joint</right_joint>
      <wheel_separation>{sep:.4f}</wheel_separation>
      <wheel_separation>{sep:.4f}</wheel_separation>
      <wheel_diameter>{dia:.4f}</wheel_diameter>
      <wheel_diameter>{dia:.4f}</wheel_diameter>
      <max_wheel_torque>{torque:.1f}</max_wheel_torque>
      <max_wheel_acceleration>20.0</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
    </plugin>
  </gazebo>

  <gazebo>
    <plugin name="tracked_joint_state" filename="libgazebo_ros_joint_state_publisher.so">
      <update_rate>30</update_rate>
      <joint_name>wheel_left_front_joint</joint_name>
      <joint_name>wheel_left_rear_joint</joint_name>
      <joint_name>wheel_right_front_joint</joint_name>
      <joint_name>wheel_right_rear_joint</joint_name>
    </plugin>
  </gazebo>

  <gazebo reference="base_scan">
    <material>Gazebo/FlatBlack</material>
    <sensor name="lidar_sensor" type="ray">
      <always_on>true</always_on>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1.000000</resolution>
            <min_angle>0.000000</min_angle>
            <max_angle>6.280000</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>{scan_min:.4f}</min>
          <max>12.0</max>
          <resolution>0.015</resolution>
        </range>
        <noise>
          <type>gaussian</type><mean>0.0</mean><stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
        <ros><remapping>~/out:=scan</remapping></ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>base_scan</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <ros><remapping>~/out:=imu</remapping></ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
    </sensor>
  </gazebo>

</robot>
'''
    info = dict(k=k, L=L, W=W, H=H, mass=m_body, sep=sep, dia=dia,
                lidar_h=base_z + lz, speed=speed, torque=torque,
                foot_l=FOOT_L * k, foot_w=FOOT_W * k, infl=INFLATION * k)
    return urdf, info


def main():
    k = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    urdf, i = build(k)

    out = os.path.expanduser('~/drone_project/urdf/tracked_bot.urdf')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as f:
        f.write(urdf)

    # 검증
    import xml.etree.ElementTree as ET
    r = ET.parse(out).getroot()
    links = [l.get('name') for l in r.findall('link')]
    joints = r.findall('joint')
    ch = set(j.find('child').get('link') for j in joints)
    root = [l for l in links if l not in ch]
    p = [x for x in r.iter('plugin') if 'diff_drive' in x.get('filename')][0]
    n = int(p.find('num_wheel_pairs').text)
    nj = len(p.findall('left_joint')) + len(p.findall('right_joint'))

    print(f"생성 완료: {out}")
    print(f"  배율 k = {i['k']}")
    print(f"  차체    {i['L']:.3f} x {i['W']:.3f} x {i['H']:.3f} m")
    print(f"  질량    {i['mass']:.2f} kg   토크 {i['torque']:.1f} N.m")
    print(f"  트랙간격 {i['sep']:.3f} m   구동륜지름 {i['dia']:.3f} m")
    print(f"  라이다  지상 {i['lidar_h']:.3f} m")
    print()
    print(f"  검증: 링크 {len(links)}개, 조인트 {len(joints)}개, 루트 {root}")
    print(f"  검증: num_wheel_pairs={n}, joints={nj} -> "
          f"{'OK' if nj == 2 * n else 'MISMATCH'}")
    print()
    print("=" * 62)
    print("Nav2 설정도 같이 바꿔야 함. 아래를 그대로 실행:")
    print("=" * 62)
    fp = (f'[[{i["foot_l"]:.3f},{i["foot_w"]:.3f}],'
          f'[{i["foot_l"]:.3f},-{i["foot_w"]:.3f}],'
          f'[-{i["foot_l"]:.3f},-{i["foot_w"]:.3f}],'
          f'[-{i["foot_l"]:.3f},{i["foot_w"]:.3f}]]')
    print("cd ~/drone_project/config")
    print(f"sed -i '195c\\      footprint: \"{fp}\"' nav2_explore.yaml")
    print(f"sed -i '234c\\      footprint: \"{fp}\"' nav2_explore.yaml")
    print(f"sed -i '200c\\        inflation_radius: {i['infl']:.3f}' nav2_explore.yaml")
    print(f"sed -i '258c\\        inflation_radius: {i['infl']:.3f}' nav2_explore.yaml")
    print(f"sed -i '146c\\      max_vel_x: {i['speed']:.2f}' nav2_explore.yaml")
    print(f"sed -i '150c\\      max_speed_xy: {i['speed']:.2f}' nav2_explore.yaml")
    print(f"sed -i '343c\\    max_velocity: [{i['speed']:.2f}, 0.0, 0.8]' nav2_explore.yaml")
    print(f"sed -i '344c\\    min_velocity: [-{i['speed']:.2f}, 0.0, -0.8]' nav2_explore.yaml")
    print("python3 -c \"import yaml; yaml.safe_load(open('nav2_explore.yaml')); print('YAML OK')\"")


if __name__ == '__main__':
    main()
