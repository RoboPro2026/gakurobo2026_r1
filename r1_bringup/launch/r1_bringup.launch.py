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
    param_file = os.path.join(pkg_dir, "config", "r1_machine_config.yaml")

    ps4_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        arguments=["--ros-args", "--log-level", "warn"],
    )

    # メカナムホイールの指令値を知りたいときはinfoにする
    r1_mecanum_node = Node(
        package="r1_machine",
        executable="r1_mecanum_node",
        name="r1_mecanum_node",  # YAMLファイル内のノード名と一致させる
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    # オドメトリの値を知りたいときはinfoにする
    r1_odometry_node = Node(
        package="r1_machine",
        executable="r1_odometry_node",
        name="r1_odometry_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    r1_sabacan_msgs_converter_node = Node(
        package="r1_machine",
        executable="r1_sabacan_msgs_converter_node",
        name="r1_sabacan_msgs_converter_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    r1_linear_motion_node = Node(
        package="r1_machine",
        executable="r1_linear_motion_node",
        name="r1_linear_motion_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "info"],
    )

    r1_state_machine_node = Node(
        package="r1_state_machine",
        executable="r1_state_machine_node",
        name="r1_state_machine_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "info"],
    )

    sabacan_robomas_node1 = Node(
        package="sabacan",
        executable="sabacan_robomasv2_node",
        name="sabacan_robomasv2_node_id1",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    sabacan_robomas_node2 = Node(
        package="sabacan",
        executable="sabacan_robomasv2_node",
        name="sabacan_robomasv2_node_id2",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    sabacan_gpio_node1 = Node(
        package="sabacan",
        executable="sabacan_gpio_node",
        name="sabacan_gpio_node_id1",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    sabacan_single_control_mecanum_fl = Node(
        package="sabacan_single_control",
        executable="sabacan_single_control_node",
        name="sabacan_single_control_mecanum_fl",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    sabacan_single_control_mecanum_fr = Node(
        package="sabacan_single_control",
        executable="sabacan_single_control_node",
        name="sabacan_single_control_mecanum_fr",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    sabacan_single_control_mecanum_rl = Node(
        package="sabacan_single_control",
        executable="sabacan_single_control_node",
        name="sabacan_single_control_mecanum_rl",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    sabacan_single_control_mecanum_rr = Node(
        package="sabacan_single_control",
        executable="sabacan_single_control_node",
        name="sabacan_single_control_mecanum_rr",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    sabacan_single_control_linear_motion = Node(
        package="sabacan_single_control",
        executable="sabacan_single_control_node",
        name="sabacan_single_control_linear_motion",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    socket_can_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros2_socketcan"),
                        "launch",
                        "socket_can_bridge.launch.xml",
                    ]
                )
            ]
        ),
        launch_arguments={"interface": "can0"}.items(),
    )

    return LaunchDescription(
        [
            ps4_node,
            r1_mecanum_node,
            r1_odometry_node,
            r1_linear_motion_node,
            r1_sabacan_msgs_converter_node,
            r1_state_machine_node,
            sabacan_robomas_node1,
            sabacan_robomas_node2,
            sabacan_gpio_node1,
            sabacan_single_control_mecanum_fl,
            sabacan_single_control_mecanum_fr,
            sabacan_single_control_mecanum_rl,
            sabacan_single_control_mecanum_rr,
            sabacan_single_control_linear_motion,
            # socket_can_bridge_launch,
        ]
    )
