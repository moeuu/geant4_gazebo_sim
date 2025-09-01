from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    origin_x     = LaunchConfiguration('origin_x')
    origin_y     = LaunchConfiguration('origin_y')
    resolution   = LaunchConfiguration('resolution')
    width        = LaunchConfiguration('width')
    height       = LaunchConfiguration('height')
    odom_topic   = LaunchConfiguration('odom_topic')
    edep_topic   = LaunchConfiguration('edep_topic')
    edep_topic_f64 = LaunchConfiguration('edep_topic_f64')
    grid_topic   = LaunchConfiguration('grid_topic')
    rate_hz      = LaunchConfiguration('publish_rate_hz')
    decay        = LaunchConfiguration('decay_factor')
    frame_id     = LaunchConfiguration('frame_id')

    rviz_config = PathJoinSubstitution([
        get_package_share_directory('g4_mapper'),
        'rviz',
        'edep_grid.rviz'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('origin_x', default_value='0.0'),
        DeclareLaunchArgument('origin_y', default_value='0.0'),
        DeclareLaunchArgument('resolution', default_value='0.25'),
        DeclareLaunchArgument('width', default_value='40'),
        DeclareLaunchArgument('height', default_value='40'),
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
                'origin_x': origin_x,
                'origin_y': origin_y,
                'resolution': resolution,
                'width': width,
                'height': height,
                'odom_topic': odom_topic,
                'edep_topic': edep_topic,
                'edep_topic_f64': edep_topic_f64,
                'grid_topic': grid_topic,
                'publish_rate_hz': rate_hz,
                'decay_factor': decay,
                'frame_id': frame_id,
            }]
        ),

        # RViz を snap/core20 から隔離しつつ、GUI必要情報は継承
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            additional_env={
                'PATH': '/usr/bin:/bin:/opt/ros/jazzy/bin',
                'LD_LIBRARY_PATH': '/opt/ros/jazzy/lib:/usr/lib/x86_64-linux-gnu',
                'QT_PLUGIN_PATH': '/opt/ros/jazzy/lib/qt/plugins',
                'AMENT_PREFIX_PATH': '/opt/ros/jazzy',
                'QT_QPA_PLATFORM': 'xcb',
                'GDK_BACKEND': 'x11',
                'GTK_MODULES': '',
                'GTK_PATH': '',
                'XDG_DATA_DIRS': '/usr/share',
                'LD_PRELOAD': ''
                # DISPLAY / HOME / XDG_RUNTIME_DIR / ROS_HOME / ROS_LOG_DIR は親環境を継承
            }
        )
    ])
