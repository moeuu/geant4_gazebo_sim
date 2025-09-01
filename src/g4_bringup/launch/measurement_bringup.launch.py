#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
measurement_bringup.launch.py  (final)

Gazebo（サーバ/GUI）+ DiffBotスポーン + ros_gz_bridge +
Geant4埋め込みノード + ロボット計測ノード + 可視化(g4_viz) を一括起動。

型安全:
- source.position / detector.position は list[float] で渡す（geant4_embed が double_array を要求）
- waypoints_yaml は与えたときだけ「文字列」で渡す
- waypoints_flat / rotation_angles は与えたときだけ list[float] に変換して渡す

/odom が出ない問題に対応:
- ブリッジを /model/diffbot/odometry (gz.msgs.Odometry) に対応
- 併せて /model/diffbot/odometry_with_covariance も接続（存在すれば同時に /odom に流す）
"""

import os
import ast
from typing import List, Optional

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable,
    IncludeLaunchDescription, TimerAction, OpaqueFunction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ---------- utils ----------

def _arg(name: str, default: str, desc: str = ""):
    return DeclareLaunchArgument(
        name,
        default_value=TextSubstitution(text=default),
        description=desc,
    )

def _as_bool(s: str) -> bool:
    return str(s).strip().lower() in ("1", "true", "yes", "on")

def _as_float(s: str) -> float:
    return float(str(s).strip())

def _parse_float_list_str(s: str) -> Optional[List[float]]:
    """空文字は None、非空は "[1,2,...]" を list[float] にして返す"""
    if s is None:
        return None
    s = s.strip()
    if s == "":
        return None
    arr = ast.literal_eval(s)  # 安全なリテラル評価
    if not isinstance(arr, list):
        raise ValueError("expected JSON-like list string, e.g. [1.0,2.0,3.0]")
    return [float(x) for x in arr]


# ---------- opaque setup ----------

def _opaque_setup(context, *args, **kwargs):
    g4_viz_share     = get_package_share_directory('g4_viz')
    g4_bringup_share = get_package_share_directory('g4_bringup')

    def LC(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    # Gazebo
    world    = LC('world')
    headless = LC('headless')
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '-v', '4', world],
        output='screen',
        condition=IfCondition(LaunchConfiguration('headless')),
    )
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-g', '-v', '4', world],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )

    # DiffBot スポーン
    diffbot_sdf = os.path.join(g4_bringup_share, 'robots', 'diffbot.sdf')
    spawn_diffbot = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/world/minimal/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--req', f'sdf_filename: "{diffbot_sdf}" name: "diffbot" allow_renaming: false',
                ],
                output='screen',
            )
        ],
    )

    # ros_gz_bridge
    #   - odometry（非 covariance）と odometry_with_covariance の両方に対応
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

            # Odometry（非 covariance）
            '/model/diffbot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # 存在する環境ではこちらを使う場合もある
            '/model/diffbot/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance',

            # cmd_vel
            '/model/diffbot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        ],
        remappings=[
            ('/model/diffbot/odometry', '/odom'),
            ('/model/diffbot/odometry_with_covariance', '/odom'),
            ('/model/diffbot/cmd_vel', '/cmd_vel'),
        ],
    )

    # Geant4 パラメータ（型整形）
    src_pos = _parse_float_list_str(LC('source_position')) or [0.0, 0.0, 0.0]
    det_pos = _parse_float_list_str(LC('detector_position')) or [0.0, 0.0, 0.0]
    g4_params = {
        'use_sim_time'             : True,
        'source.position'          : src_pos,
        'source.intensity'         : _as_float(LC('source_intensity')),
        'detector.position'        : det_pos,
        'shield.angle_deg'         : _as_float(LC('shield_angle_deg')),
        'noise.enabled'            : _as_bool(LC('noise_enabled')),
        'noise.strength'           : _as_float(LC('noise_strength')),
        'beam.particle'            : LC('particle'),
        'energy.offset_mev'        : _as_float(LC('energy_offset_mev')),
        'energy.scale_mev_per_mps' : _as_float(LC('energy_scale_mev_per_mps')),
        'energy.max_mev'           : _as_float(LC('energy_max_mev')),
    }
    geant4_node = Node(
        package='geant4_embed',
        executable='geant4_embed_node',
        name='g4_odom_subscriber',
        output='screen',
        parameters=[g4_params],
    )

    # ロボット計測ノード（空文字は渡さない、与える時だけ正しい型）
    rm_params = {
        'use_sim_time' : True,
        'linear_speed' : _as_float(LC('linear_speed')),
        'angular_speed': _as_float(LC('angular_speed')),
        'pos_tolerance': _as_float(LC('pos_tolerance')),
        'measure_delay': _as_float(LC('measure_delay')),
    }
    wp_yaml = LC('waypoints_yaml').strip()
    if wp_yaml != '':
        rm_params['waypoints_yaml'] = wp_yaml
    wp_flat = _parse_float_list_str(LC('waypoints_flat'))
    if wp_flat is not None:
        rm_params['waypoints_flat'] = wp_flat
    rot = _parse_float_list_str(LC('rotation_angles'))
    if rot is not None:
        rm_params['rotation_angles'] = rot

    robot_measure_node = Node(
        package='g4_bringup',
        executable='robot_measurement_node',
        name='robot_measurement_node',
        output='screen',
        parameters=[rm_params],
    )

    # 可視化(g4_viz)
    edep_plotter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(g4_viz_share, 'launch', 'edep_plotter.launch.py')),
        condition=IfCondition(LaunchConfiguration('plot')),
        launch_arguments={
            'edep_max': LaunchConfiguration('edep_max'),
            'csv_dir' : LaunchConfiguration('csv_dir'),
            'topic'   : '/g4/edep',
        }.items(),
    )
    edep_hist = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(g4_viz_share, 'launch', 'edep_hist.launch.py')),
        condition=IfCondition(LaunchConfiguration('hist')),
        launch_arguments={
            'edep_max': LaunchConfiguration('edep_max'),
            'csv_dir' : LaunchConfiguration('csv_dir'),
            'bins'    : '50',
            'topic'   : '/g4/edep',
        }.items(),
    )
    edep_grid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(g4_viz_share, 'launch', 'edep_grid.launch.py')),
        condition=IfCondition(LaunchConfiguration('grid')),
        launch_arguments={
            'resolution_m': LaunchConfiguration('resolution_m'),
            'range_m'     : LaunchConfiguration('range_m'),
            'decay_alpha' : LaunchConfiguration('decay_alpha'),
            'mode'        : LaunchConfiguration('mode'),
            'edep_max'    : LaunchConfiguration('edep_max'),
            'csv_dir'     : LaunchConfiguration('csv_dir'),
            'odom_topic'  : '/odom',
            'edep_topic'  : '/g4/edep',
            'pub_rate_hz' : '5.0',
        }.items(),
    )

    return [
        gz_server, gz_gui,
        spawn_diffbot,
        bridge,
        geant4_node,
        robot_measure_node,
        edep_plotter, edep_hist, edep_grid,
    ]


# ---------- main LaunchDescription ----------

def generate_launch_description() -> LaunchDescription:
    g4_viz_share     = get_package_share_directory('g4_viz')
    g4_bringup_share = get_package_share_directory('g4_bringup')

    default_world = os.path.join(g4_viz_share, 'worlds', 'minimal.sdf')

    # 基本
    world_arg    = _arg('world', default_world, 'SDF world file')
    headless_arg = _arg('headless', 'true', 'true: server only, false: GUI')

    # Geant4
    src_pos_arg  = _arg('source_position', '[0.0, 0.0, 0.0]', 'source [x,y,z] as JSON list')
    src_int_arg  = _arg('source_intensity', '1.0', 'source intensity')
    det_pos_arg  = _arg('detector_position', '[0.0, 0.0, 0.0]', 'detector [x,y,z] as JSON list')
    shield_arg   = _arg('shield_angle_deg', '0.0', 'shield angle [deg]')
    noise_en_arg = _arg('noise_enabled', 'false', 'enable noise')
    noise_st_arg = _arg('noise_strength', '0.0', 'noise stddev')
    particle_arg = _arg('particle', 'gamma', 'beam particle')
    e_off_arg    = _arg('energy_offset_mev', '0.5', 'E = off + scl*v')
    e_scl_arg    = _arg('energy_scale_mev_per_mps', '2.0', 'scale [MeV/(m/s)]')
    e_max_arg    = _arg('energy_max_mev', '10.0', 'E max [MeV]')

    # Robot
    wp_yaml_arg  = _arg('waypoints_yaml', '',  'waypoints JSON/YAML string (parsed by node)')
    wp_flat_arg  = _arg('waypoints_flat', '',  'flat waypoints string, e.g. "[1.0,1.0,2.0,2.0]"')
    rot_arg      = _arg('rotation_angles', '', 'rotation angles string, e.g. "[0,90,180,270]"')
    lin_arg      = _arg('linear_speed', '0.5', 'linear speed [m/s]')
    ang_arg      = _arg('angular_speed', '1.0', 'angular speed [rad/s]')
    tol_arg      = _arg('pos_tolerance', '0.05', 'goal tolerance [m]')
    del_arg      = _arg('measure_delay', '2.0', 'measure delay [s]')

    # 可視化
    edep_max_arg = _arg('edep_max', '1.0', 'Edep visualize upper bound')
    plot_arg     = _arg('plot', 'true', 'enable plotter')
    hist_arg     = _arg('hist', 'true', 'enable histogram')
    grid_arg     = _arg('grid', 'true', 'enable occupancy grid')
    csv_dir_arg  = _arg('csv_dir', os.path.join(os.environ.get('HOME',''), 'g4_ros_ws', 'results'), 'CSV output dir')
    res_arg      = _arg('resolution_m', '0.25', 'grid resolution [m]')
    rng_arg      = _arg('range_m', '10.0', 'grid range [m]')
    dec_arg      = _arg('decay_alpha', '1.0', 'grid decay alpha')
    mode_arg     = _arg('mode', 'sum', 'grid mode (sum|max)')

    # GZ リソースパス
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    add_paths = f"{g4_viz_share}:{g4_bringup_share}"
    merged = f"{add_paths}:{existing}" if existing else add_paths
    set_gz_env = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=merged)

    return LaunchDescription([
        world_arg, headless_arg,
        src_pos_arg, src_int_arg, det_pos_arg, shield_arg,
        noise_en_arg, noise_st_arg, particle_arg,
        e_off_arg, e_scl_arg, e_max_arg,
        wp_yaml_arg, wp_flat_arg, rot_arg,
        lin_arg, ang_arg, tol_arg, del_arg,
        edep_max_arg, plot_arg, hist_arg, grid_arg,
        csv_dir_arg, res_arg, rng_arg, dec_arg, mode_arg,
        set_gz_env,
        OpaqueFunction(function=_opaque_setup),
    ])
