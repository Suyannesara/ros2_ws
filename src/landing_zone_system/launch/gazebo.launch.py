# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# import os
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     # Caminho completo para o mundo
#     world_file = os.path.join(
#         get_package_share_directory('landing_zone_system'),
#         'worlds', 'grass.world'  # ou 'bayland.sdf'
#     )

#     return LaunchDescription([
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(
#                     get_package_share_directory('gazebo_ros'),
#                     'launch', 'gazebo.launch.py'
#                 )
#             ),
#             launch_arguments={'world': world_file}.items()
#         )
#     ])

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Caminho absoluto para o mundo customizado
    pkg_share = get_package_share_directory('landing_zone_system')
    world_file = os.path.join(pkg_share, 'worlds', 'grass.world')

    # Caminho do PX4 (ajuste conforme seu ambiente)
    px4_path = os.path.expanduser('~/PX4-Autopilot')

    return LaunchDescription([
        # 1) Executa o PX4 SITL com Gazebo Classic e mundo customizado
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'export GAZEBO_WORLD={world_file} && make px4_sitl gazebo-classic_iris_depth_camera'
            ],
            cwd=px4_path,
            output='screen',
            shell=True,
        ),

        # 2) Espera 10s para PX4 e Gazebo iniciarem e executa seu nó ROS
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
