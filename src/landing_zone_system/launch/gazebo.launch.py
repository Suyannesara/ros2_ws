from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = os.path.join(
        get_package_share_directory('landing_zone_system'),
        'worlds', 'grass.world')
    
#     world_file = os.path.join(
# +        get_package_share_directory('gazebo_ros'),
# +        'worlds', 'garden.world')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                  get_package_share_directory('gazebo_ros'),
                  'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': pkg}.items()
        )
    ])

