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
    TextSubstitution,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    g4_viz_share = get_package_share_directory('g4_viz')
    default_world_path = os.path.join(g4_viz_share, 'worlds', 'minimal.sdf')

    world_arg = DeclareLaunchArgument('world', default_value=TextSubstitution(text=default_world_path))
    headless_arg = DeclareLaunchArgument('headless', default_value='true')

    particle_arg = DeclareLaunchArgument('particle', default_value='gamma')
    noise_enabled_arg  = DeclareLaunchArgument('noise_enabled',  default_value='true')
    noise_strength_arg = DeclareLaunchArgument('noise_strength', default_value='0.5')

    src_pos_arg   = DeclareLaunchArgument('source_position',   default_value='[0.0, 0.0, 0.0]')
    src_int_arg   = DeclareLaunchArgument('source_intensity',  default_value='1.0')
    det_pos_arg   = DeclareLaunchArgument('detector_position', default_value='[0.0, 0.0, 0.0]')
    shield_arg    = DeclareLaunchArgument('shield_angle_deg',  default_value='0.0')

    e_offset_arg = DeclareLaunchArgument('energy_offset_mev',        default_value='0.5')
    e_scale_arg  = DeclareLaunchArgument('energy_scale_mev_per_mps', default_value='2.0')
    e_max_arg    = DeclareLaunchArgument('energy_max_mev',           default_value='10.0')

    edep_max_arg = DeclareLaunchArgument('edep_max', default_value='1.0')
    plot_arg = DeclareLaunchArgument('plot', default_value='true')
    hist_arg = DeclareLaunchArgument('hist', default_value='true')
    grid_arg = DeclareLaunchArgument('grid', default_value='true')
    csv_dir_arg = DeclareLaunchArgument(
        'csv_dir',
        default_value=TextSubstitution(text=os.path.join(os.environ.get('HOME', ''), 'g4_ros_ws', 'results'))
    )
    resolution_arg = DeclareLaunchArgument('resolution_m', default_value='0.25')
    range_arg      = DeclareLaunchArgument('range_m',      default_value='10.0')
    decay_arg      = DeclareLaunchArgument('decay_alpha',  default_value='1.0')
    mode_arg       = DeclareLaunchArgument('mode',         default_value='sum')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')

    particle = LaunchConfiguration('particle')
    noise_enabled  = LaunchConfiguration('noise_enabled')
    noise_strength = LaunchConfiguration('noise_strength')

    src_pos   = LaunchConfiguration('source_position')
    src_int   = LaunchConfiguration('source_intensity')
    det_pos   = LaunchConfiguration('detector_position')
    shield    = LaunchConfiguration('shield_angle_deg')

    e_offset = LaunchConfiguration('energy_offset_mev')
    e_scale  = LaunchConfiguration('energy_scale_mev_per_mps')
    e_max    = LaunchConfiguration('energy_max_mev')

    edep_max = LaunchConfiguration('edep_max')
    plot = LaunchConfiguration('plot')
    hist = LaunchConfiguration('hist')
    grid = LaunchConfiguration('grid')
    csv_dir = LaunchConfiguration('csv_dir')
    res_m = LaunchConfiguration('resolution_m')
    rng_m = LaunchConfiguration('range_m')
    decay_a = LaunchConfiguration('decay_alpha')
    mode = LaunchConfiguration('mode')

    g4_bringup_share = get_package_share_directory('g4_bringup')
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    add_paths = f"{g4_viz_share}:{g4_bringup_share}"
    merged_gz_path = f"{add_paths}:{existing_gz_path}" if existing_gz_path else add_paths
    set_gz_env = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=merged_gz_path)

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

    diffbot_sdf = os.path.join(g4_bringup_share, 'robots', 'diffbot.sdf')
    spawn_diffbot_cmd = (
        f'gz service -s /world/minimal/create '
        f'--reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean '
        f'--req \'sdf_filename: "{diffbot_sdf}" name: "diffbot" allow_renaming: false\''
    )
    spawn_diffbot = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(cmd=['bash', '-lc', spawn_diffbot_cmd], output='screen')]
    )

    # ★ ここを修正：with_cov と plain の両方をブリッジ
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/model/diffbot/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance',
        '/model/diffbot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry]',
        '/model/diffbot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        ],
        remappings=[
            ('/model/diffbot/odometry_with_covariance', '/odom'),
            ('/model/diffbot/odometry', '/odom'),
            ('/model/diffbot/cmd_vel', '/cmd_vel'),
        ],

    )

    geant4_node = Node(
        package='geant4_embed',
        executable='geant4_embed_node',
        name='g4_odom_subscriber',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'source.position': src_pos,
            'source.intensity': ParameterValue(src_int, value_type=float),
            'detector.position': det_pos,
            'shield.angle_deg': ParameterValue(shield, value_type=float),
            'noise.enabled': ParameterValue(noise_enabled, value_type=bool),
            'noise.strength': ParameterValue(noise_strength, value_type=float),
            'beam.particle': particle,
            'energy.offset_mev': ParameterValue(e_offset, value_type=float),
            'energy.scale_mev_per_mps': ParameterValue(e_scale, value_type=float),
            'energy.max_mev': ParameterValue(e_max, value_type=float),
        }],
    )

    include_plotter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(g4_viz_share, 'launch', 'edep_plotter.launch.py')),
        condition=IfCondition(plot),
        launch_arguments={'edep_max': edep_max, 'csv_dir': csv_dir, 'topic': '/g4/edep'}.items()
    )
    include_hist = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(g4_viz_share, 'launch', 'edep_hist.launch.py')),
        condition=IfCondition(hist),
        launch_arguments={'edep_max': edep_max, 'csv_dir': csv_dir, 'bins': '50', 'topic': '/g4/edep'}.items()
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
        world_arg, headless_arg,
        particle_arg, noise_enabled_arg, noise_strength_arg,
        src_pos_arg, src_int_arg, det_pos_arg, shield_arg,
        e_offset_arg, e_scale_arg, e_max_arg,
        edep_max_arg, plot_arg, hist_arg, grid_arg, csv_dir_arg,
        resolution_arg, range_arg, decay_arg, mode_arg,
        set_gz_env,
        gz_server, gz_gui,
        spawn_diffbot,
        bridge,
        geant4_node,
        include_plotter,
        include_hist,
        include_grid,
    ])

