from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    
    red_detector = Node(
        package='red_detector',
        executable='red_detector',
        output='screen'
    )
    
    red_obj_server = Node(
        package='red_detector',
        executable='red_obj_server',
        output='screen'
    )
    
    # 创建一个5秒的定时器动作
    delay_timer = TimerAction(
        period=5.0,
        actions=[red_obj_server]
    )
    
    return LaunchDescription([
        red_detector,
        delay_timer
    ])