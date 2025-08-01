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
            ]
            # parameters=[
            #     {'origem': 'ros'},
            #     {'cameraTopic': '/camera/image_raw'},
            #     {'tempoBuscaSegundos': 10},
            #     {'width': 640},
            #     {'height': 480},
            #     {'fps': 30.0},
            #     {'gridRows': 6},
            #     {'gridCols': 8},
            #     {'salvarVideo': True},
            #     {'saidaVideoPath': '/home/suyanne/ros2_ws/output.avi'},
            # ]
        )
    ])
