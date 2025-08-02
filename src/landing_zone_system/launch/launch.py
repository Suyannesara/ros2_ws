# from launch import LaunchDescription
# from launch.actions import TimerAction
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# import os
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     pkg_share = get_package_share_directory('landing_zone_system')
#     bayland_world = os.path.join(pkg_share, 'worlds', 'grass.sdf')

#     gazebo_ros_pkg_share = get_package_share_directory('gazebo_ros')

#     return LaunchDescription([
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(gazebo_ros_pkg_share, 'launch', 'gazebo.launch.py')
#             ),
#             launch_arguments={'world': bayland_world}.items(),
#         ),

#         TimerAction(
#             period=8.0,  # espera o gazebo abrir
#             actions=[
#                 Node(
#                     package='landing_zone_system',
#                     executable='landing_zone_node',
#                     output='screen',
#                     parameters=[
#                         {'camera_topic': '/camera/image_raw'},
#                         {'width': 1280},
#                         {'height': 960},
#                         {'gridRows': 6},
#                         {'gridCols': 8},
#                     ]
#                 )
#             ]
#         )
#     ])
