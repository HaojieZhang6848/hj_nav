from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():
    
    display_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')]
        )
    )
    
    return LaunchDescription([
        display_nav_launch
    ])