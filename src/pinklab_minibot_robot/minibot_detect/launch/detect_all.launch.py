import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription() # Launch파일 생성
    detect_all_pkg = get_package_share_directory('minibot_detect')

    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
        detect_all_pkg,
        'param',
        'config.yaml'
        )
        )
    args = [DeclareLaunchArgument(
        'param_dir',
        default_value=param_dir
    )]

    aruco_detect = Node(
        package='minibot_detect',
        executable='aruco_detect',
        parameters=[param_dir],
        output='screen'
    )

    object_detect = Node(
        package='minibot_detect',
        executable='object_detect',
        parameters=[param_dir],
        output='screen'
    )

    # all_detect = Node(
    #     package='minibot_detect',
    #     executable='all_detect',
    #     parameters=[param_dir]
    #     output='screen'
    # )

    ld = LaunchDescription(args) # launch파일 생성
    ld.add_action(aruco_detect)
    ld.add_action(object_detect)
    # ld.add_action(all_detect)
    return ld
    