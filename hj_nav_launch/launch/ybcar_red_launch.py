from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    red_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('red_detector'), 'launch', 'start.launch.py')]
        )
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='/camera/color'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='image_raw'
    )
    
    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('apriltag_ros'), 'launch', 'tag_36h11_all.launch.py')
        ),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'image_topic': LaunchConfiguration('image_topic'),
        }.items(),
    )

    return LaunchDescription([
        red_detector_launch,
        camera_name_arg,
        image_topic_arg,
        apriltag_launch
    ])
