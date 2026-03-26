import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_sabacan_robomasv2_node(
    param_file, board_id: int, log_level: str = "warn"
) -> Node:
    # 基板ごとに node 名と reset service 名だけを切り替えて使い回す。
    return Node(
        package="sabacan",
        executable="sabacan_robomasv2_node",
        name=f"sabacan_robomasv2_node_id{board_id}",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("sabacan_robomas_reset", f"sabacan_robomas_reset_id{board_id}"),
        ],
    )


def create_sabacan_single_control_node(
    board_id: int, motor_number: int, control_cycle: float, log_level: str = "warn"
) -> Node:
    # single_control_node は 1 軸専用なので、board と motor を明示して生成する。
    return Node(
        package="sabacan_single_control",
        executable="sabacan_single_control_node",
        name=f"sabacan_single_control_node_id{board_id}_motor{motor_number}",
        parameters=[
            {
                "board_id": board_id,
                "motor_number": motor_number,
                "control_cycle": control_cycle,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )


def generate_launch_description():
    pkg_dir = get_package_share_directory("r1_bringup")
    default_param_file = os.path.join(pkg_dir, "config", "test_config.yaml")

    param_file = LaunchConfiguration("param_file")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        arguments=["--ros-args", "--log-level", "warn"],
    )

    r1_swerve_drive_node = Node(
        package="r1_machine",
        executable="r1_swerve_drive_node",
        name="r1_swerve_drive_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "info"],
    )

    test_node = Node(
        package="r1_main",
        executable="test_node",
        name="test_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "info"],
    )
    # test_node は Joy 入力、cmd_vel 生成、Sabacan single ref 変換をまとめて担う。

    sabacan_power_node_id0 = Node(
        package="sabacan",
        executable="sabacan_power_node",
        name="sabacan_power_node_id0",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    eth2can_node = Node(
        package="eth2can",
        executable="eth2can_node",
        name="eth2can_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        remappings=[
            ("from_can_bus0", "from_can_bus"),
            ("to_can_bus0", "to_can_bus"),
        ],
    )

    # Sabacan 基板ノード群は test_bringup で常時起動する。
    hardware_core_nodes = [
        sabacan_power_node_id0,
        create_sabacan_robomasv2_node(param_file, 1),
        create_sabacan_robomasv2_node(param_file, 2),
    ]

    # single_control は 1 モータ 1 ノード構成なので、足回り 8 軸分を並べて起動する。
    hardware_single_control_nodes = [
        create_sabacan_single_control_node(1, 0, 100.0),
        create_sabacan_single_control_node(1, 1, 100.0),
        create_sabacan_single_control_node(1, 2, 100.0),
        create_sabacan_single_control_node(1, 3, 100.0),
        create_sabacan_single_control_node(2, 0, 100.0),
        create_sabacan_single_control_node(2, 1, 100.0),
        create_sabacan_single_control_node(2, 2, 100.0),
        create_sabacan_single_control_node(2, 3, 100.0),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("param_file", default_value=default_param_file),
            joy_node,
            r1_swerve_drive_node,
            test_node,
            eth2can_node,
            # 先に CAN 基板ノードを起動して、status / service を先に立ち上げる。
            TimerAction(
                period=1.0,
                actions=hardware_core_nodes,
            ),
            # その後で single_control を起動し、各軸の ref 購読先を揃える。
            TimerAction(
                period=2.0,
                actions=hardware_single_control_nodes,
            ),
        ]
    )
