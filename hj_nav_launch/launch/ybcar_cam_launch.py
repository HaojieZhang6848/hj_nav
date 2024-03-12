from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    camera_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('astra_camera'), 'launch', 'astro_pro_plus.launch.xml')]
        )
    )
    
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{'port': 8080}]
    )

    return LaunchDescription([
        web_video_server_node,
        camera_launch
    ])
