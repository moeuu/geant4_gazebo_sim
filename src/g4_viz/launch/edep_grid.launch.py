from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    resolution_m = LaunchConfiguration('resolution_m', default='0.25')
    range_m = LaunchConfiguration('range_m', default='10.0')
    decay_alpha = LaunchConfiguration('decay_alpha', default='1.0')
    mode = LaunchConfiguration('mode', default='sum')
    edep_max = LaunchConfiguration('edep_max', default='1.0')
    csv_dir = LaunchConfiguration('csv_dir', default='')
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')
    edep_topic = LaunchConfiguration('edep_topic', default='/g4/edep')
    pub_rate_hz = LaunchConfiguration('pub_rate_hz', default='5.0')
    return LaunchDescription([
        DeclareLaunchArgument('resolution_m', default_value='0.25'),
        DeclareLaunchArgument('range_m', default_value='10.0'),
        DeclareLaunchArgument('decay_alpha', default_value='1.0'),
        DeclareLaunchArgument('mode', default_value='sum'),
        DeclareLaunchArgument('edep_max', default_value='1.0'),
        DeclareLaunchArgument('csv_dir', default_value=''),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('edep_topic', default_value='/g4/edep'),
        DeclareLaunchArgument('pub_rate_hz', default_value='5.0'),
        Node(
            package='g4_viz',
            executable='edep_grid_node',
            name='edep_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'resolution_m': resolution_m,
                'range_m': range_m,
                'decay_alpha': decay_alpha,
                'mode': mode,
                'edep_max': edep_max,
                'csv_dir': csv_dir,
                'odom_topic': odom_topic,
                'edep_topic': edep_topic,
                'pub_rate_hz': pub_rate_hz,
            }]
        )
    ])
