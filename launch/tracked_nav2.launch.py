#!/usr/bin/env python3
# ==========================================================
# tracked_nav2.launch.py
#   spartina_nav2.launch.py 의 궤도차 버전.
#   Gazebo + 갯벌 월드 + tracked_bot 소환 + static TF 2개(map->odom, map->drone_camera_optical)
# ==========================================================

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---- 경로 ----
    home = os.path.expanduser('~')
    proj = os.path.join(home, 'drone_project')

    urdf_file = os.path.join(proj, 'urdf', 'tracked_bot.urdf')
    world_file = os.path.join(proj, 'worlds', 'spartina_world.world')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ---- 스폰 위치 (기존과 동일하게 -2, 0) ----
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # ---- Gazebo 서버/클라이언트 ----
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # ---- robot_state_publisher ----
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
        }],
    )

    # ---- 로봇 소환 (robot_description 토픽에서 읽음) ----
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_tracked_bot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'tracked_bot',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.05',
        ],
    )

    # ---- static TF: map -> odom (identity) ----
    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---- static TF: map -> drone_camera_optical (상공 8m, 수직 하방) ----
    # 쿼터니언 (x,y,z,w) = (0.707107, -0.707107, 0, 0)  ← 기존 런치와 동일
    tf_map_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_camera',
        output='screen',
        arguments=['0', '0', '8',
                   '0.707107', '-0.707107', '0', '0',
                   'map', 'drone_camera_optical'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        gzserver,
        gzclient,
        rsp,
        tf_map_odom,
        tf_map_camera,
        spawn_robot,
    ])
