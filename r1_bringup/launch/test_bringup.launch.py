import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # パッケージの共有ディレクトリのパスを取得
    pkg_dir = get_package_share_directory("r1_bringup")

    # パラメータファイルのフルパスを作成
    param_file = os.path.join(pkg_dir, "config", "test_config.yaml")

    r1_chassis_control_node = Node(
        package="r1_control",
        executable="r1_chassis_control_node",
        name="r1_chassis_control_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription(
        [
            r1_chassis_control_node
            # socket_can_bridge_launch,
        ]
    )
