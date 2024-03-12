from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare the launch arguments
    color_roi_x_arg = DeclareLaunchArgument('color_roi_x', default_value='0')
    color_roi_y_arg = DeclareLaunchArgument('color_roi_y', default_value='0')
    color_roi_width_arg = DeclareLaunchArgument('color_roi_width', default_value='640')
    color_roi_height_arg = DeclareLaunchArgument('color_roi_height', default_value='400')
    
    # Get argument values
    color_roi_x = LaunchConfiguration('color_roi_x')
    color_roi_y = LaunchConfiguration('color_roi_y')
    color_roi_width = LaunchConfiguration('color_roi_width')
    color_roi_height = LaunchConfiguration('color_roi_height')

    camera_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('astra_camera'), 'launch', 'dabai_u3.launch.xml')]
        ),
        launch_arguments={
            'color_roi_x': color_roi_x,
            'color_roi_y': color_roi_y,
            'color_roi_width': color_roi_width,
            'color_roi_height': color_roi_height
        }.items()
    )
    
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{'port': 8080}]
    )

    return LaunchDescription([
        color_roi_x_arg,
        color_roi_y_arg,
        color_roi_width_arg,
        color_roi_height_arg,
        web_video_server_node,
        camera_launch
    ])
