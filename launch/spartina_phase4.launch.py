"""
spartina_phase4.launch.py — Phase 4 통합 런치 v4 (완전 자동 시나리오)
=====================================================================
터미널 1개로 전체 시나리오가 올바른 순서로 흐른다:

  런치 실행 (탐사 범위·복귀 좌표를 인자로 지정)
  → Gazebo + Nav2 + 탐지 파이프라인 기동 (전부 대기 상태)
  → (20초) 🚁 드론이 지정 범위를 지그재그 "한 바퀴" 탐사
  → 이때부터 target_manager 가 타겟 등록 (require_survey:=true)
  → 🚁 탐사 완료 → 이륙 지점 복귀·정지 (DONE 발행)
  → 🚗 로봇이 DONE 을 확인한 뒤에야 출발 (wait_survey_done:=true)
  → 발견된 기둥 순차 방문 → 지정 좌표(home_x, home_y)로 복귀 → 미션 완료

[기동 시간표]
   0초 : Gazebo + 로봇 + TF (spartina_multi.launch.py include)
  10초 : Nav2
  12초 : locator, 탐지기, target_manager, mission_manager (전부 대기 진입)
  20초 : drone_survey — 여기서부터 시나리오가 "시작" 된다

[v4 수정]
  🚁 드론: drone_survey_node(무한 반복) → drone_survey_once_node(1회+복귀+정지)
  🚗 로봇: mission_manager 에 wait_survey_done:=true — 드론 DONE 후 출발

[사용 — 인자로 시나리오 지정]
  export TURTLEBOT3_MODEL=waffle && export ROS_LOCALHOST_ONLY=1

  # 기본값 (탐사 ±3m, 복귀 (-3,-3)):
  ros2 launch ~/drone_project/launch/spartina_phase4.launch.py

  # 범위·복귀 좌표 지정:
  ros2 launch ~/drone_project/launch/spartina_phase4.launch.py \\
    x_min:=-3.0 x_max:=3.0 y_min:=-3.0 y_max:=3.0 \\
    home_x:=-3.0 home_y:=-3.0

  ※ 복귀 좌표는 기둥((2,0),(-2,2),(0,-2))·장애물((0,0),(3,-1.5),(3.5,1.5))과
    1m 이상 떨어진 빈 곳으로.

[개별 실행과의 사용 구분]
  - 전체 시나리오 시연/확인 → 이 런치 (터미널 1개)
  - 특정 노드만 고치고 재시작하는 개발 작업 → 개별 7터미널 (실행가이드 §4)

담당: 여수환 · 스마트해운물류 x ICT 멘토링 (파트 3) · Phase 4
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HOME = os.path.expanduser('~')
PROJECT = os.path.join(HOME, 'drone_project')
SCRIPTS = os.path.join(PROJECT, 'scripts')


def launch_setup(context):
    """런치 인자값을 문자열로 확정한 뒤 노드들을 조립한다 (v3 핵심)."""
    def val(name):
        return LaunchConfiguration(name).perform(context)

    x_min, x_max = val('x_min'), val('x_max')
    y_min, y_max = val('y_min'), val('y_max')
    lane, alt = val('lane_spacing'), val('altitude')
    speed, hold = val('speed'), val('hold_sec')
    home_x, home_y = val('home_x'), val('home_y')

    # ── 10초: Nav2 ──
    nav2 = TimerAction(period=10.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    os.environ.get('NAV2_BRINGUP_DIR',
                                   '/opt/ros/humble/share/nav2_bringup'),
                    'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': os.path.join(PROJECT, 'config',
                                            'nav2_explore.yaml'),
            }.items()),
    ])

    # ── 12초: 탐지 파이프라인 + 매니저 (전부 대기 상태로 진입) ──
    pipeline = TimerAction(period=12.0, actions=[
        # 좌표 계산 (locator multi)
        Node(
            executable='python3',
            arguments=[os.path.join(SCRIPTS, 'obstacle_locator_multi_node.py'),
                       '--ros-args',
                       '-p', 'use_tf2:=true',
                       '-p', 'world_frame:=map',
                       '-p', 'camera_frame:=drone_camera_optical',
                       '-p', 'robot_frame:=base_link',
                       '-p', 'use_sim_time:=true',
                       '-p', 'lookup_timeout:=0.0'],   # ★ 기아 방지 (가이드 §3-②)
            output='screen'),
        # 탐지기 (multi)
        Node(
            executable='python3',
            arguments=[os.path.join(SCRIPTS, 'spartina_detector_multi_node.py'),
                       '--ros-args',
                       '-p', 'use_sim_time:=true',
                       '-p', 'show_window:=true'],
            output='screen'),
        # target_manager — 탐사 시작 전엔 탐지 무시 (v2 require_survey)
        Node(
            executable='python3',
            arguments=[os.path.join(SCRIPTS, 'target_manager_node.py'),
                       '--ros-args',
                       '-p', 'use_sim_time:=true',
                       '-p', 'require_survey:=true'],
            output='screen'),
        # mission_manager — 확정 3개 대기, 완료 후 지정 좌표로 복귀
        Node(
            executable='python3',
            arguments=[os.path.join(SCRIPTS, 'mission_manager_node.py'),
                       '--ros-args',
                       '-p', 'use_sim_time:=true',
                       '-p', 'wait_survey_done:=true',   # [v4] 🚁 탐사 완료 후 출발
                       '-p', 'use_home_pose:=true',
                       '-p', f'home_pose:=[{home_x}, {home_y}]'],
            output='screen'),
    ])

    # ── 20초: 🚁 드론 1회 탐사 시작 — 여기서부터 시나리오 개시 ──
    survey = TimerAction(period=20.0, actions=[
        Node(
            executable='python3',
            arguments=[os.path.join(SCRIPTS, 'drone_survey_once_node.py'),
                       '--ros-args',
                       '-p', 'use_sim_time:=true',
                       '-p', f'area_x_min:={x_min}',
                       '-p', f'area_x_max:={x_max}',
                       '-p', f'area_y_min:={y_min}',
                       '-p', f'area_y_max:={y_max}',
                       '-p', f'lane_spacing:={lane}',
                       '-p', f'altitude:={alt}',
                       '-p', f'speed:={speed}',
                       '-p', f'hold_sec:={hold}'],
            output='screen'),
    ])

    return [nav2, pipeline, survey]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('x_min', default_value='-3.0'),
        DeclareLaunchArgument('x_max', default_value='3.0'),
        DeclareLaunchArgument('y_min', default_value='-3.0'),
        DeclareLaunchArgument('y_max', default_value='3.0'),
        DeclareLaunchArgument('lane_spacing', default_value='3.0'),
        DeclareLaunchArgument('altitude', default_value='8.0'),
        DeclareLaunchArgument('speed', default_value='0.8'),
        DeclareLaunchArgument('hold_sec', default_value='3.0'),
        DeclareLaunchArgument('home_x', default_value='-3.0'),
        DeclareLaunchArgument('home_y', default_value='-3.0'),
    ]

    # ── 0초: Gazebo(multi 월드) + 로봇 + static TF + drone_tf_broadcaster ──
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(PROJECT, 'launch', 'spartina_multi.launch.py')))

    return LaunchDescription(args + [base, OpaqueFunction(function=launch_setup)])
