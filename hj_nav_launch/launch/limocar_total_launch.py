from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    
    base_2_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.1', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link'],
    )
    
    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('limo_bringup'), 'launch', 'limo_start.launch.py')]
        )
    )

    navigation_dwa_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('limo_bringup'), 'launch', 'navigation2.launch.py')]
        )
    )

    nav_server_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory('nav_http_server'), 'launch', 'start.launch.py')]
                )
            )
        ]
    )
    
    red_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory('hj_nav_launch'), 'launch', 'limocar_red_launch.py')]
                )
            )
        ]
    )

    return LaunchDescription([
        base_2_camera,
        navigation_dwa_launch,
        laser_bringup_launch,
        nav_server_launch,
        red_launch
    ])
