"""
🎭 [무대 런치] spartina_phase4_base.launch.py — 인프라만 기동
=============================================================
"무대" 와 "공연" 을 분리한다:

  무대 (이 런치, 한 번만): Gazebo + 로봇 + TF + Nav2 + locator + 탐지기
                           + target_manager + mission_manager (전부 대기 상태)
  공연 (run_scenario.sh, 몇 번이든): 리셋 신호 → 🚁 드론 탐사 실행
      → 탐사 완료(DONE) → 🚗 로봇 미션 자동 진행

무대를 켜둔 채 run_scenario.sh 의 인자(범위·속도·복귀 좌표)만 바꿔
반복 실행할 수 있다 — Gazebo 재시작 불필요.

[기동 시간표]
   0초 : Gazebo + 로봇 + TF (spartina_multi.launch.py include)
  10초 : Nav2
  12초 : locator, 탐지기, target_manager, mission_manager (전부 대기)
  ※ 드론 탐사는 포함하지 않는다 — run_scenario.sh 가 실행.

사용:
  export TURTLEBOT3_MODEL=waffle && export ROS_LOCALHOST_ONLY=1
  ros2 launch ~/drone_project/launch/spartina_phase4_base.launch.py
  # 이후: bash ~/drone_project/scripts/run_scenario.sh [인자들]

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

    home_x, home_y = val('home_x'), val('home_y')
    # 탐사 관련 인자(범위·속도)는 run_scenario.sh 가 드론 노드에 직접 전달

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
                       '-p', 'target_height:=1.0',
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

    return [nav2, pipeline]


def generate_launch_description():
    # home_x/y 는 무대 기동 시의 "기본" 복귀 좌표.
    # 공연마다 바꾸려면 run_scenario.sh 의 인자가 우선한다
    # (스크립트가 mission_manager 파라미터를 런타임에 갱신).
    args = [
        DeclareLaunchArgument('home_x', default_value='-3.0'),
        DeclareLaunchArgument('home_y', default_value='-3.0'),
    ]

    # ── 0초: Gazebo(multi 월드) + 로봇 + static TF + drone_tf_broadcaster ──
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(PROJECT, 'launch', 'arena01_multi.launch.py')))

    return LaunchDescription(args + [base, OpaqueFunction(function=launch_setup)])
