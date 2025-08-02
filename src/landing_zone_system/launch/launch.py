from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Pega caminho absoluto do mundo custom
    pkg_share = get_package_share_directory('landing_zone_system')
    world_file = os.path.join(pkg_share, 'worlds', 'grass.world')

    # Caminho da pasta PX4
    px4_path = os.path.expanduser('~/PX4-Autopilot')

    return LaunchDescription([
        # 1) Roda PX4 SITL SEM Gazebo
        ExecuteProcess(
            cmd=['make', 'px4_sitl', 'none'],
            cwd=px4_path,
            output='screen',
        ),

        # 2) Espera PX4 iniciar e roda Gazebo Classic com seu mundo
        TimerAction(
            period=5.0,  # espera 5 seg pra PX4 iniciar
            actions=[
                ExecuteProcess(
                    cmd=['gazebo', '--verbose', world_file],
                    output='screen',
                )
            ]
        ),

        # 3) Espera tudo iniciar e roda seu node ROS (exemplo)
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='landing_zone_system',
                    executable='landing_zone_node',
                    output='screen',
                    parameters=[
                        {'camera_topic': '/camera/image_raw'},
                        {'width': 1280},
                        {'height': 960},
                        {'gridRows': 6},
                        {'gridCols': 8},
                    ],
                )
            ]
        )
    ])
