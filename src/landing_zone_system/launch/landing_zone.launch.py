from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='landing_zone_system',
            executable='landing_zone_node',
            name='landing_zone_node',
            output='screen',
            parameters=[
                {'camera_topic': '/camera/image_raw'},
                {'width': 1280},
                {'height': 960},
                {'gridRows': 6},
                {'gridCols': 8},
                {'tracker_type': 'template'}
            ]
        )
    ])
