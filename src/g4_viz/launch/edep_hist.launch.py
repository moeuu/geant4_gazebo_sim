from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    edep_max = LaunchConfiguration('edep_max', default='1.0')
    csv_dir = LaunchConfiguration('csv_dir', default='')
    bins = LaunchConfiguration('bins', default='50')
    topic = LaunchConfiguration('topic', default='/g4/edep')
    return LaunchDescription([
        DeclareLaunchArgument('edep_max', default_value='1.0'),
        DeclareLaunchArgument('csv_dir', default_value=''),
        DeclareLaunchArgument('bins', default_value='50'),
        DeclareLaunchArgument('topic', default_value='/g4/edep'),
        Node(
            package='g4_viz',
            executable='edep_hist_node',
            name='edep_hist_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'edep_max': edep_max,
                'csv_dir': csv_dir,
                'bins': bins,
                'topic': topic,
            }]
        )
    ])
