from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():
    
    ybcar_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('hj_nav_launch'), 'launch', 'ybcar_nav_launch.py')]
        )
    )
    
    red_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('hj_nav_launch'), 'launch', 'ybcar_red_launch.py')]
        )
    )
    
    cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('hj_nav_launch'), 'launch', 'ybcar_cam_launch.py')]
        )
    )
    
    return LaunchDescription([
        ybcar_nav_launch,
        red_launch,
        cam_launch
    ])