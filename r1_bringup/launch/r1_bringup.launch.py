import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # パッケージの共有ディレクトリのパスを取得
    pkg_dir = get_package_share_directory("r1_bringup")

    # パラメータファイルのフルパスを作成
    param_file = os.path.join(pkg_dir, "config", "r1_machine_config.yaml")

    # TODO: 次はros2_socketcanとsabacanのlaunchを書くところから。

    r1_mecanum_node = Node(
        package="r1_machine",
        executable="r1_mecanum_node",
        name="r1_mecanum_node",  # YAMLファイル内のノード名と一致させる
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    r1_state_machine_node = Node(
        package="r1_state_machine",
        executable="r1_state_machine_node",
        name="r1_state_machine_node",
        arguments=["--ros-args", "--log-level", "warn"],
    )

    return LaunchDescription([r1_mecanum_node, r1_state_machine_node])
