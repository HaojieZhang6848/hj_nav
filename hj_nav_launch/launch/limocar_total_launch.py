from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ruamel.yaml import YAML

INITIAL_POSE_X = os.getenv('INITIAL_POSE_X')
INITIAL_POSE_Y = os.getenv('INITIAL_POSE_Y')
INITIAL_POSE_YAW = os.getenv('INITIAL_POSE_YAW')

SET_INITIAL_POSE = True if INITIAL_POSE_X and INITIAL_POSE_Y and INITIAL_POSE_YAW else False
if SET_INITIAL_POSE:
    INITIAL_POSE_X = float(INITIAL_POSE_X)
    INITIAL_POSE_Y = float(INITIAL_POSE_Y)
    INITIAL_POSE_YAW = float(INITIAL_POSE_YAW)

def get_sibling_file(file, sibling_file):
    return os.path.join(os.path.dirname(file), sibling_file)

def generate_launch_description():
    orig_param_file = os.path.join(get_package_share_directory('limo_bringup'), 'param', 'navigation2.yaml')
    # 替换掉原本的导航参数中的set_initial_pose和initial_pose_x/y/yaw，保存成新的文件
    if SET_INITIAL_POSE:
        yaml = YAML()
        with open(orig_param_file, 'r') as file:
            param = yaml.load(file)
        param['amcl']['ros__parameters']['set_initial_pose'] = SET_INITIAL_POSE
        if 'initial_pose' not in param['amcl']['ros__parameters']:
            param['amcl']['ros__parameters']['initial_pose'] = {}
        param['amcl']['ros__parameters']['initial_pose']['x'] = INITIAL_POSE_X
        param['amcl']['ros__parameters']['initial_pose']['y'] = INITIAL_POSE_Y
        param['amcl']['ros__parameters']['initial_pose']['yaw'] = INITIAL_POSE_YAW
        new_param_file = get_sibling_file(orig_param_file, 'navigation2_new.yaml')
        with open(new_param_file, 'w') as file:
            yaml.dump(param, file)
        
    
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
        ),
        launch_arguments={'params_file': get_sibling_file(orig_param_file, 'navigation2_new.yaml') if SET_INITIAL_POSE else orig_param_file}.items()
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
