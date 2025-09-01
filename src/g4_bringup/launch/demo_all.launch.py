#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import (
    LaunchConfiguration,
    EnvironmentVariable,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # ---- パス: デフォルト world は g4_viz の minimal.sdf（world名: minimal）----
    g4_viz_share = get_package_share_directory('g4_viz')
    default_world_path = os.path.join(g4_viz_share, 'worlds', 'minimal.sdf')

    # ---- 引数 ----
    world_arg = DeclareLaunchArgument('world', default_value=TextSubstitution(text=default_world_path))
    headless_arg = DeclareLaunchArgument('headless', default_value='true')

    particle_arg = DeclareLaunchArgument('particle', default_value='e-')
    edep_max_arg = DeclareLaunchArgument('edep_max', default_value='1.0')
    plot_arg = DeclareLaunchArgument('plot', default_value='true')
    hist_arg = DeclareLaunchArgument('hist', default_value='true')
    grid_arg = DeclareLaunchArgument('grid', default_value='true')
    csv_dir_arg = DeclareLaunchArgument(
        'csv_dir',
        default_value=PathJoinSubstitution([EnvironmentVariable('HOME'), 'g4_ros_ws', 'results'])
    )
    # grid 可視化
    resolution_arg = DeclareLaunchArgument('resolution_m', default_value='0.25')
    range_arg = DeclareLaunchArgument('range_m', default_value='10.0')
    decay_arg = DeclareLaunchArgument('decay_alpha', default_value='1.0')
    mode_arg = DeclareLaunchArgument('mode', default_value='sum')

    # ---- Substitutions ----
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    particle = LaunchConfiguration('particle')
    edep_max = LaunchConfiguration('edep_max')
    plot = LaunchConfiguration('plot')
    hist = LaunchConfiguration('hist')
    grid = LaunchConfiguration('grid')
    csv_dir = LaunchConfiguration('csv_dir')
    res_m = LaunchConfiguration('resolution_m')
    rng_m = LaunchConfiguration('range_m')
    decay_a = LaunchConfiguration('decay_alpha')
    mode = LaunchConfiguration('mode')

    # ---- GZ_SIM_RESOURCE_PATH ----
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    merged_gz_path = f"{g4_viz_share}:{existing_gz_path}" if existing_gz_path else g4_viz_share
    set_gz_env = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=merged_gz_path)

    # ---- Gazebo 起動（headless/GUI）----
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '-v', '4', world],
        output='screen',
        condition=IfCondition(headless)
    )
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-g', '-v', '4', world],
        output='screen',
        condition=UnlessCondition(headless)
    )

    # ---- diffbot 自動スポーン（world名は minimal 固定）----
    # 2秒待ってから /world/minimal/create に投げる
    diffbot_sdf = os.path.join(get_package_share_directory('g4_bringup'), 'robots', 'diffbot.sdf')
    spawn_diffbot_cmd = (
        f'gz service -s /world/minimal/create '
        f'--reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean '
        f'--req \'sdf_filename: "{diffbot_sdf}" name: "diffbot" allow_renaming: false\''
    )
    spawn_diffbot = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(cmd=['bash', '-lc', spawn_diffbot_cmd], output='screen')
        ]
    )

    # ---- ros_gz_bridge ----
    # /model/diffbot/odometry_with_covariance -> /odom（GZ->ROS）
    # /model/diffbot/cmd_vel <-> /cmd_vel（双方向）
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/diffbot/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance',
            '/model/diffbot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        ],
        remappings=[
            ('/model/diffbot/odometry_with_covariance', '/odom'),
            ('/model/diffbot/cmd_vel', '/cmd_vel'),
        ],
    )

    # ---- Geant4 ノード ----
    geant4_node = Node(
        package='geant4_embed',
        executable='geant4_embed_node',
        name='geant4_embed_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'particle': particle,
            'edep_max': edep_max,
            'csv_dir': csv_dir,
        }],
    )

    # ---- 可視化（g4_viz）----
    g4_viz_share = get_package_share_directory('g4_viz')
    include_plotter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(g4_viz_share, 'launch', 'edep_plotter.launch.py')),
        condition=IfCondition(plot),
        launch_arguments={
            'edep_max': edep_max,
            'csv_dir': csv_dir,
            'topic': '/g4/edep',
        }.items()
    )
    include_hist = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(g4_viz_share, 'launch', 'edep_hist.launch.py')),
        condition=IfCondition(hist),
        launch_arguments={
            'edep_max': edep_max,
            'csv_dir': csv_dir,
            'bins': '50',
            'topic': '/g4/edep',
        }.items()
    )
    include_grid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(g4_viz_share, 'launch', 'edep_grid.launch.py')),
        condition=IfCondition(grid),
        launch_arguments={
            'resolution_m': res_m,
            'range_m': rng_m,
            'decay_alpha': decay_a,
            'mode': mode,
            'edep_max': edep_max,
            'csv_dir': csv_dir,
            'odom_topic': '/odom',
            'edep_topic': '/g4/edep',
            'pub_rate_hz': '5.0',
        }.items()
    )

    return LaunchDescription([
        # Args
        world_arg, headless_arg,
        particle_arg, edep_max_arg, plot_arg, hist_arg, grid_arg, csv_dir_arg,
        resolution_arg, range_arg, decay_arg, mode_arg,

        # Env
        set_gz_env,

        # Gazebo
        gz_server, gz_gui,

        # Spawn after short delay
        spawn_diffbot,

        # Bridge & Geant4
        bridge,
        geant4_node,

        # Viz
        include_plotter,
        include_hist,
        include_grid,
    ])
