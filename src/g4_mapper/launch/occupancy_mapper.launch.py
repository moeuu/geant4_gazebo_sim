from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('origin_x', default_value='0.0'),
        DeclareLaunchArgument('origin_y', default_value='0.0'),
        DeclareLaunchArgument('resolution', default_value='0.25'),
        DeclareLaunchArgument('width', default_value='80'),
        DeclareLaunchArgument('height', default_value='80'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('edep_topic', default_value='/g4/edep'),
        DeclareLaunchArgument('edep_topic_f64', default_value='/g4/edep'),
        DeclareLaunchArgument('grid_topic', default_value='/g4/edep_grid'),
        DeclareLaunchArgument('publish_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('decay_factor', default_value='0.0'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        Node(
            package='g4_mapper',
            executable='occupancy_mapper',
            name='g4_edep_occupancy_mapper',
            output='screen',
            parameters=[{
                'origin_x': LaunchConfiguration('origin_x'),
                'origin_y': LaunchConfiguration('origin_y'),
                'resolution': LaunchConfiguration('resolution'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'edep_topic': LaunchConfiguration('edep_topic'),
                'edep_topic_f64': LaunchConfiguration('edep_topic_f64'),
                'grid_topic': LaunchConfiguration('grid_topic'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'decay_factor': LaunchConfiguration('decay_factor'),
                'frame_id': LaunchConfiguration('frame_id'),
            }]
        )
    ])
