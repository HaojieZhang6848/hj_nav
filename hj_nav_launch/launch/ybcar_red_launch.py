from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription


def generate_launch_description():

    camera_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('astra_camera'), 'launch', 'astro_pro_plus.launch.xml')]
        )
    )

    red_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('red_detector'), 'launch', 'start.launch.py')]
        )
    )

    return LaunchDescription([
        red_detector_launch,
        camera_launch
    ])
