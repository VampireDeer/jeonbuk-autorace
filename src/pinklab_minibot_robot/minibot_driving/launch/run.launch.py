import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription() #launch파일 생성
    auto_race = get_package_share_directory('minibot_driving')

    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
        auto_race,
        'param',
        'config.yaml')
        )

    args = [DeclareLaunchArgument(
        'param_dir',
        default_value=param_dir
        )]

    lane_detect = Node(
        package='minibot_driving',
        executable='lane_detect',
        parameters=[param_dir],
        output="screen"
        )

    slam_control = Node(
        package='minibot_driving',
        executable='slam_control3',
        parameters=[param_dir],
        output='screen'
        )
    
    ld = LaunchDescription(args) #laucnh파일 생성
    ld.add_action(lane_detect)
    ld.add_action(slam_control)
    return ld