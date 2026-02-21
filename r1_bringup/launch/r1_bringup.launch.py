import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


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

    bno086_node = Node(
        package="bno086",
        executable="bno086_node",
        name="bno086_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    r1_main_node = Node(
        package="r1_main",
        executable="r1_main_node",
        name="r1_main_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "info"],
    )

    r1_chassis_control_node = Node(
        package="r1_control",
        executable="r1_chassis_control_node",
        name="r1_chassis_control_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # ========== 足回り ==========
    # メカナムホイールの指令値を知りたいときはinfoにする
    r1_mecanum_node = Node(
        package="r1_machine",
        executable="r1_mecanum_node",
        name="r1_mecanum_node",  # YAMLファイル内のノード名と一致させる
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    r1_dummy_odometry_node = Node(
        package="r1_control",
        executable="r1_dummy_odometry_node",
        name="r1_dummy_odometry_node",
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

    # ========== KFS回収 ==========
    r1_kfs_fx_node = Node(
        package="r1_machine",
        executable="r1_linear_motion_node",
        name="r1_kfs_fx_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        remappings=[
            ("linear_motion_status", "kfs_fx_linear_motion_status"),
            ("linear_motion_motor_ref", "kfs_fx_motor_ref"),
            ("linear_motion_position_ref", "kfs_fx_position_ref"),
            ("linear_motion_detect_origin", "kfs_fx_detect_origin"),
            ("linear_motion_mode_status", "kfs_fx_mode_status"),
        ],
    )

    r1_kfs_fz_node = Node(
        package="r1_machine",
        executable="r1_linear_motion_node",
        name="r1_kfs_fz_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        remappings=[
            ("linear_motion_status", "kfs_fz_linear_motion_status"),
            ("linear_motion_motor_ref", "kfs_fz_motor_ref"),
            ("linear_motion_position_ref", "kfs_fz_position_ref"),
            ("linear_motion_detect_origin", "kfs_fz_detect_origin"),
            ("linear_motion_mode_status", "kfs_fz_mode_status"),
        ],
    )

    r1_kfs_fyaw_node = Node(
        package="r1_machine",
        executable="r1_angle_motion_node",
        name="r1_kfs_fyaw_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        remappings=[
            ("angle_motion_status", "kfs_fyaw_angle_motion_status"),
            ("angle_motion_motor_ref", "kfs_fyaw_motor_ref"),
            ("angle_motion_position_ref", "kfs_fyaw_position_ref"),
            ("angle_motion_detect_origin", "kfs_fyaw_detect_origin"),
            ("angle_motion_mode_status", "kfs_fyaw_mode_status"),
        ],
    )

    r1_kfs_rx_node = Node(
        package="r1_machine",
        executable="r1_linear_motion_node",
        name="r1_kfs_rx_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        remappings=[
            ("linear_motion_status", "kfs_rx_linear_motion_status"),
            ("linear_motion_motor_ref", "kfs_rx_motor_ref"),
            ("linear_motion_position_ref", "kfs_rx_position_ref"),
            ("linear_motion_detect_origin", "kfs_rx_detect_origin"),
            ("linear_motion_mode_status", "kfs_rx_mode_status"),
        ],
    )

    r1_kfs_rz_node = Node(
        package="r1_machine",
        executable="r1_linear_motion_node",
        name="r1_kfs_rz_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        remappings=[
            ("linear_motion_status", "kfs_rz_linear_motion_status"),
            ("linear_motion_motor_ref", "kfs_rz_motor_ref"),
            ("linear_motion_position_ref", "kfs_rz_position_ref"),
            ("linear_motion_detect_origin", "kfs_rz_detect_origin"),
            ("linear_motion_mode_status", "kfs_rz_mode_status"),
        ],
    )

    r1_kfs_ryaw_node = Node(
        package="r1_machine",
        executable="r1_angle_motion_node",
        name="r1_kfs_ryaw_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        remappings=[
            ("angle_motion_status", "kfs_ryaw_angle_motion_status"),
            ("angle_motion_motor_ref", "kfs_ryaw_motor_ref"),
            ("angle_motion_position_ref", "kfs_ryaw_position_ref"),
            ("angle_motion_detect_origin", "kfs_ryaw_detect_origin"),
            ("angle_motion_mode_status", "kfs_ryaw_mode_status"),
        ],
    )

    # ========== 展開 ==========
    r1_front_expand_node = Node(
        package="r1_machine",
        executable="r1_linear_motion_node",
        name="r1_front_expand_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        remappings=[
            ("linear_motion_status", "front_expand_linear_motion_status"),
            ("linear_motion_motor_ref", "front_expand_motor_ref"),
            ("linear_motion_position_ref", "front_expand_position_ref"),
            ("linear_motion_detect_origin", "front_expand_detect_origin"),
            ("linear_motion_mode_status", "front_expand_mode_status"),
        ],
    )

    r1_rear_expand_node = Node(
        package="r1_machine",
        executable="r1_linear_motion_node",
        name="r1_rear_expand_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        remappings=[
            ("linear_motion_status", "rear_expand_linear_motion_status"),
            ("linear_motion_motor_ref", "rear_expand_motor_ref"),
            ("linear_motion_position_ref", "rear_expand_position_ref"),
            ("linear_motion_detect_origin", "rear_expand_detect_origin"),
            ("linear_motion_mode_status", "rear_expand_mode_status"),
        ],
    )

    # ========== ポール ==========
    r1_pole_x1_node = Node(
        package="r1_machine",
        executable="r1_linear_motion_node",
        name="r1_pole_x1_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        remappings=[
            ("linear_motion_status", "pole_x1_linear_motion_status"),
            ("linear_motion_motor_ref", "pole_x1_motor_ref"),
            ("linear_motion_position_ref", "pole_x1_position_ref"),
            ("linear_motion_detect_origin", "pole_x1_detect_origin"),
            ("linear_motion_mode_status", "pole_x1_mode_status"),
        ],
    )
    r1_pole_x2_node = Node(
        package="r1_machine",
        executable="r1_linear_motion_node",
        name="r1_pole_x2_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        remappings=[
            ("linear_motion_status", "pole_x2_linear_motion_status"),
            ("linear_motion_motor_ref", "pole_x2_motor_ref"),
            ("linear_motion_position_ref", "pole_x2_position_ref"),
            ("linear_motion_detect_origin", "pole_x2_detect_origin"),
            ("linear_motion_mode_status", "pole_x2_mode_status"),
        ],
    )

    r1_pole_y_node = Node(
        package="r1_machine",
        executable="r1_linear_motion_node",
        name="r1_pole_y_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        remappings=[
            ("linear_motion_status", "pole_y_linear_motion_status"),
            ("linear_motion_motor_ref", "pole_y_motor_ref"),
            ("linear_motion_position_ref", "pole_y_position_ref"),
            ("linear_motion_detect_origin", "pole_y_detect_origin"),
            ("linear_motion_mode_status", "pole_y_mode_status"),
        ],
    )

    r1_pole_roger_node = Node(
        package="r1_machine",
        executable="r1_linear_motion_node",
        name="r1_pole_roger_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        remappings=[
            ("linear_motion_status", "pole_roger_linear_motion_status"),
            ("linear_motion_motor_ref", "pole_roger_motor_ref"),
            ("linear_motion_position_ref", "pole_roger_position_ref"),
            ("linear_motion_detect_origin", "pole_roger_detect_origin"),
            ("linear_motion_mode_status", "pole_roger_mode_status"),
        ],
    )

    r1_spear_roger1_node = Node(
        package="r1_machine",
        executable="r1_linear_motion_node",
        name="r1_spear_roger1_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        remappings=[
            ("linear_motion_status", "spear_roger1_linear_motion_status"),
            ("linear_motion_motor_ref", "spear_roger1_motor_ref"),
            ("linear_motion_position_ref", "spear_roger1_position_ref"),
            ("linear_motion_detect_origin", "spear_roger1_detect_origin"),
            ("linear_motion_mode_status", "spear_roger1_mode_status"),
        ],
    )

    r1_spear_roger2_node = Node(
        package="r1_machine",
        executable="r1_linear_motion_node",
        name="r1_spear_roger2_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        remappings=[
            ("linear_motion_status", "spear_roger2_linear_motion_status"),
            ("linear_motion_motor_ref", "spear_roger2_motor_ref"),
            ("linear_motion_position_ref", "spear_roger2_position_ref"),
            ("linear_motion_detect_origin", "spear_roger2_detect_origin"),
            ("linear_motion_mode_status", "spear_roger2_mode_status"),
        ],
    )

    r1_spear_move_node = Node(
        package="r1_machine",
        executable="r1_linear_motion_node",
        name="r1_spear_move_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        remappings=[
            ("linear_motion_status", "spear_move_linear_motion_status"),
            ("linear_motion_motor_ref", "spear_move_motor_ref"),
            ("linear_motion_position_ref", "spear_move_position_ref"),
            ("linear_motion_detect_origin", "spear_move_detect_origin"),
            ("linear_motion_mode_status", "spear_move_mode_status"),
            ("low_switch_status", "spear_move_switch_status"),
        ],
    )

    r1_spear_rotate_node = Node(
        package="r1_machine",
        executable="r1_angle_motion_node",
        name="r1_spear_rotate_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        remappings=[
            ("angle_motion_status", "spear_rotate_angle_motion_status"),
            ("angle_motion_motor_ref", "spear_rotate_motor_ref"),
            ("angle_motion_position_ref", "spear_rotate_position_ref"),
            ("angle_motion_detect_origin", "spear_rotate_detect_origin"),
            ("angle_motion_mode_status", "spear_rotate_mode_status"),
            ("low_switch_status", "spear_rotate_switch_status"),
        ],
    )

    def create_sabacan_robomasv2_node(board_id: int, log_level="warn") -> Node:
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

    sabacan_robomasv2_node_id1 = create_sabacan_robomasv2_node(1)
    sabacan_robomasv2_node_id2 = create_sabacan_robomasv2_node(2)
    sabacan_robomasv2_node_id3 = create_sabacan_robomasv2_node(3)
    sabacan_robomasv2_node_id4 = create_sabacan_robomasv2_node(4)
    sabacan_robomasv2_node_id5 = create_sabacan_robomasv2_node(5)
    sabacan_robomasv2_node_id6 = create_sabacan_robomasv2_node(6)

    def create_sabacan_single_control_node(
        board_id: int, motor_number: int, log_level="warn"
    ) -> Node:
        return Node(
            package="sabacan_single_control",
            executable="sabacan_single_control_node",
            name=f"sabacan_single_control_node_id{board_id}_motor{motor_number}",
            parameters=[
                # param_file,
                {"board_id": board_id, "motor_number": motor_number},
            ],
            arguments=["--ros-args", "--log-level", log_level],
        )

    sabacan_single_control_id1_motor0 = create_sabacan_single_control_node(1, 0)
    sabacan_single_control_id1_motor1 = create_sabacan_single_control_node(1, 1)
    sabacan_single_control_id1_motor2 = create_sabacan_single_control_node(1, 2)
    sabacan_single_control_id1_motor3 = create_sabacan_single_control_node(1, 3)
    sabacan_single_control_id2_motor0 = create_sabacan_single_control_node(2, 0)
    sabacan_single_control_id2_motor1 = create_sabacan_single_control_node(2, 1)
    sabacan_single_control_id2_motor2 = create_sabacan_single_control_node(2, 2)
    sabacan_single_control_id2_motor3 = create_sabacan_single_control_node(2, 3)
    sabacan_single_control_id3_motor0 = create_sabacan_single_control_node(3, 0)
    sabacan_single_control_id3_motor1 = create_sabacan_single_control_node(3, 1)
    sabacan_single_control_id3_motor2 = create_sabacan_single_control_node(3, 2)
    sabacan_single_control_id3_motor3 = create_sabacan_single_control_node(3, 3)
    sabacan_single_control_id4_motor0 = create_sabacan_single_control_node(4, 0)
    sabacan_single_control_id4_motor1 = create_sabacan_single_control_node(4, 1)
    sabacan_single_control_id4_motor2 = create_sabacan_single_control_node(4, 2)
    sabacan_single_control_id4_motor3 = create_sabacan_single_control_node(4, 3)
    sabacan_single_control_id5_motor0 = create_sabacan_single_control_node(5, 0)
    sabacan_single_control_id5_motor1 = create_sabacan_single_control_node(5, 1)
    sabacan_single_control_id5_motor2 = create_sabacan_single_control_node(5, 2)
    sabacan_single_control_id5_motor3 = create_sabacan_single_control_node(5, 3)
    sabacan_single_control_id6_motor0 = create_sabacan_single_control_node(6, 0)
    sabacan_single_control_id6_motor1 = create_sabacan_single_control_node(6, 1)
    sabacan_single_control_id6_motor2 = create_sabacan_single_control_node(6, 2)
    sabacan_single_control_id6_motor3 = create_sabacan_single_control_node(6, 3)

    def create_sabacan_gpio_node(board_id: int, log_level="warn") -> Node:
        return Node(
            package="sabacan",
            executable="sabacan_gpio_node",
            name=f"sabacan_gpio_node_id{board_id}",
            parameters=[param_file],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=[
                ("sabacan_gpio_reset", f"sabacan_gpio_reset_id{board_id}"),
            ],
        )

    sabacan_gpio_node_id1 = create_sabacan_gpio_node(1)
    sabacan_gpio_node_id2 = create_sabacan_gpio_node(2)
    sabacan_gpio_node_id3 = create_sabacan_gpio_node(3)

    sabacan_power_node_id0 = Node(
        package="sabacan",
        executable="sabacan_power_node",
        name="sabacan_power_node_id0",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    sabacan_led_node_id1 = Node(
        package="sabacan",
        executable="sabacan_led_node",
        name="sabacan_led_node_id1",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    socket_can_bridge_launch = GroupAction(
        [
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ros2_socketcan"),
                            "launch",
                            "socket_can_bridge.launch.xml",
                        ]
                    )
                ),
                launch_arguments={"interface": "can0"}.items(),
            ),
        ]
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        # output="screen",
        parameters=[
            {
                "port": 8765,
            }
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    # r1_mainのノードの起動を遅延させる
    normal_nodes = [
        foxglove_node,
        ps4_node,
        bno086_node,
        r1_chassis_control_node,
        r1_mecanum_node,
        # r1_dummy_odometry_node,
        r1_odometry_node,
        r1_sabacan_msgs_converter_node,
        r1_kfs_fx_node,
        r1_kfs_fz_node,
        r1_kfs_fyaw_node,
        r1_kfs_rx_node,
        r1_kfs_rz_node,
        r1_kfs_ryaw_node,
        r1_front_expand_node,
        r1_rear_expand_node,
        r1_pole_x1_node,
        r1_pole_x2_node,
        r1_pole_y_node,
        r1_pole_roger_node,
        r1_spear_roger1_node,
        r1_spear_roger2_node,
        r1_spear_move_node,
        r1_spear_rotate_node,
        sabacan_single_control_id1_motor0,
        sabacan_single_control_id1_motor1,
        sabacan_single_control_id1_motor2,
        sabacan_single_control_id1_motor3,
        sabacan_single_control_id2_motor0,
        sabacan_single_control_id2_motor1,
        sabacan_single_control_id2_motor2,
        sabacan_single_control_id2_motor3,
        sabacan_single_control_id3_motor0,
        sabacan_single_control_id3_motor1,
        sabacan_single_control_id3_motor2,
        sabacan_single_control_id3_motor3,
        sabacan_single_control_id4_motor0,
        sabacan_single_control_id4_motor1,
        sabacan_single_control_id4_motor2,
        sabacan_single_control_id4_motor3,
        sabacan_single_control_id5_motor0,
        sabacan_single_control_id5_motor1,
        sabacan_single_control_id5_motor2,
        sabacan_single_control_id5_motor3,
        sabacan_single_control_id6_motor0,
        sabacan_single_control_id6_motor1,
        sabacan_single_control_id6_motor2,
        sabacan_single_control_id6_motor3,
        sabacan_robomasv2_node_id1,
        sabacan_robomasv2_node_id2,
        sabacan_robomasv2_node_id3,
        sabacan_robomasv2_node_id4,
        sabacan_robomasv2_node_id5,
        sabacan_robomasv2_node_id6,
        sabacan_gpio_node_id1,
        sabacan_gpio_node_id2,
        sabacan_gpio_node_id3,
        sabacan_power_node_id0,
        sabacan_led_node_id1,
    ]

    # sabacanは遅延させて起動
    return LaunchDescription(
        [
            TimerAction(period=0.0, actions=normal_nodes),
            TimerAction(period=2.0, actions=[r1_main_node]),
        ]
    )
