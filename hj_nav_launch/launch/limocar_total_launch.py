from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('hj_nav_launch'), 'launch', 'limocar_nav_launch.py')]
        )
    )
    
    red_launch =IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('hj_nav_launch'), 'launch', 'limocar_red_launch.py')]
        )
    )

    return LaunchDescription([
        nav_launch,
        red_launch
    ])
