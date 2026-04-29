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
from launch.conditions import IfCondition, UnlessCondition
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
    use_sim = LaunchConfiguration("use_sim")
    use_lidar = LaunchConfiguration("use_lidar")
    use_aruco_display = LaunchConfiguration("use_aruco_display")
    robot_control_mode = LaunchConfiguration("robot_control_mode")

    # パラメータファイルのフルパスを作成
    param_file = os.path.join(pkg_dir, "config", "r1_machine_config.yaml")
    zone_parameter = {"zone": "blue"}
    robot_control_mode_parameter = {"robot_control_mode": robot_control_mode}

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
        parameters=[param_file, zone_parameter, robot_control_mode_parameter],
        arguments=["--ros-args", "--log-level", "info"],
    )

    r1_chassis_control_node = Node(
        package="r1_control",
        executable="r1_chassis_control_node",
        name="r1_chassis_control_node",
        parameters=[param_file, zone_parameter],
        arguments=["--ros-args", "--log-level", "info"],
    )

    r1_chassis_velocity_control_node = Node(
        package="r1_control",
        executable="r1_chassis_velocity_control_node",
        name="r1_chassis_velocity_control_node",
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

    # サーボの指令値を知りたいときはinfoにする
    r1_swerve_drive_node = Node(
        package="r1_machine",
        executable="r1_swerve_drive_node",
        name="r1_swerve_drive_node",
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

    r1_machine_manage_node = Node(
        package="r1_machine",
        executable="r1_machine_manage_node",
        name="r1_machine_manage_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "info"],
    )

    def create_r1_linear_motion_node(
        node_name: str, topic_prefix: str, extra_remappings=None, log_level="warn"
    ) -> Node:
        remappings = [
            ("linear_motion_status", f"{topic_prefix}_linear_motion_status"),
            ("linear_motion_motor_ref", f"{topic_prefix}_motor_ref"),
            ("linear_motion_position_ref", f"{topic_prefix}_position_ref"),
            ("linear_motion_speed_ref", f"{topic_prefix}_speed_ref"),
            ("linear_motion_speed_mode_stop", f"{topic_prefix}_speed_mode_stop"),
            ("linear_motion_detect_origin", f"{topic_prefix}_detect_origin"),
            ("linear_motion_initialize", f"{topic_prefix}_initialize"),
            ("linear_motion_set_pos", f"{topic_prefix}_set_pos"),
            ("linear_motion_mode_status", f"{topic_prefix}_mode_status"),
            ("linear_motion_torque_limit_ref", f"{topic_prefix}_torque_limit_ref"),
            ("linear_motion_move_mech_lock", f"{topic_prefix}_move_mech_lock"),
        ]
        if extra_remappings is not None:
            remappings.extend(extra_remappings)

        return Node(
            package="r1_machine",
            executable="r1_linear_motion_node",
            name=node_name,
            parameters=[param_file],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings,
        )

    def create_r1_angle_motion_node(
        node_name: str, topic_prefix: str, extra_remappings=None, log_level="warn"
    ) -> Node:
        remappings = [
            ("angle_motion_status", f"{topic_prefix}_angle_motion_status"),
            ("angle_motion_motor_ref", f"{topic_prefix}_motor_ref"),
            ("angle_motion_position_ref", f"{topic_prefix}_position_ref"),
            ("angle_motion_speed_ref", f"{topic_prefix}_speed_ref"),
            ("angle_motion_speed_mode_stop", f"{topic_prefix}_speed_mode_stop"),
            ("angle_motion_detect_origin", f"{topic_prefix}_detect_origin"),
            ("angle_motion_initialize", f"{topic_prefix}_initialize"),
            ("angle_motion_set_angle", f"{topic_prefix}_set_angle"),
            ("angle_motion_mode_status", f"{topic_prefix}_mode_status"),
            ("angle_motion_torque_limit_ref", f"{topic_prefix}_torque_limit_ref"),
            ("angle_motion_move_mech_lock", f"{topic_prefix}_move_mech_lock"),
        ]
        if extra_remappings is not None:
            remappings.extend(extra_remappings)

        return Node(
            package="r1_machine",
            executable="r1_angle_motion_node",
            name=node_name,
            parameters=[param_file],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings,
        )

    # ========== KFS回収 ==========
    r1_kfs_fx_node = create_r1_linear_motion_node("r1_kfs_fx_node", "kfs_fx")
    r1_kfs_fz_node = create_r1_linear_motion_node(
        "r1_kfs_fz_node",
        "kfs_fz",
        extra_remappings=[("low_switch_status", "kfs_fz_low_switch_status")],
    )
    r1_kfs_fyaw_node = create_r1_angle_motion_node("r1_kfs_fyaw_node", "kfs_fyaw")
    r1_r2_flift_node = create_r1_linear_motion_node(
        "r1_r2_flift_node",
        "r2_flift",
    )
    r1_kfs_rx_node = create_r1_linear_motion_node("r1_kfs_rx_node", "kfs_rx")
    r1_kfs_rz_node = create_r1_linear_motion_node(
        "r1_kfs_rz_node",
        "kfs_rz",
        extra_remappings=[("low_switch_status", "kfs_rz_low_switch_status")],
    )
    r1_r2_rlift_node = create_r1_linear_motion_node(
        "r1_r2_rlift_node",
        "r2_rlift",
    )
    r1_kfs_ryaw_node = create_r1_angle_motion_node("r1_kfs_ryaw_node", "kfs_ryaw")

    # ========== やり ==========
    # 大槻機構
    r1_spear_y_node = create_r1_linear_motion_node(
        "r1_spear_y_node",
        "spear_y",
        extra_remappings=[("low_switch_status", "spear_y_low_switch_status")],
    )

    r1_spear_roll_node = create_r1_angle_motion_node("r1_spear_roll_node", "spear_roll")

    # 千田機構
    # r1_spear1_node = create_r1_linear_motion_node(
    #     "r1_spear1_node",
    #     "spear1",
    #     extra_remappings=[("low_switch_status", "spear1_low_switch_status")],
    # )
    # r1_spear2_node = create_r1_linear_motion_node(
    #     "r1_spear2_node",
    #     "spear2",
    #     extra_remappings=[("low_switch_status", "spear2_low_switch_status")],
    # )
    # r1_spear3_node = create_r1_linear_motion_node(
    #     "r1_spear3_node",
    #     "spear3",
    #     extra_remappings=[("low_switch_status", "spear3_low_switch_status")],
    # )
    # r1_spear4_node = create_r1_linear_motion_node(
    #     "r1_spear4_node",
    #     "spear4",
    #     extra_remappings=[("low_switch_status", "spear4_low_switch_status")],
    # )
    # r1_spear_x_node = create_r1_linear_motion_node(
    #     "r1_spear_x_node",
    #     "spear_x",
    #     extra_remappings=[("low_switch_status", "spear_x_low_switch_status")],
    # )
    # r1_spear_y_node = create_r1_linear_motion_node(
    #     "r1_spear_y_node",
    #     "spear_y",
    #     extra_remappings=[("low_switch_status", "spear_y_low_switch_status")],
    # )
    # r1_spear_roll_node = create_r1_angle_motion_node("r1_spear_roll_node", "spear_roll")
    # r1_spear_pitch1_node = create_r1_angle_motion_node(
    #     "r1_spear_pitch1_node", "spear_pitch1"
    # )
    # r1_spear_pitch2_node = create_r1_angle_motion_node(
    #     "r1_spear_pitch2_node", "spear_pitch2"
    # )

    def create_sabacan_robomasv2_node(
        board_id: int,
        from_can_bus="from_can_bus0",
        to_can_bus="to_can_bus0",
        log_level="warn",
    ) -> Node:
        return Node(
            package="sabacan",
            executable="sabacan_robomasv2_node",
            name=f"sabacan_robomasv2_node_id{board_id}",
            parameters=[param_file],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=[
                ("from_can_bus", from_can_bus),
                ("to_can_bus", to_can_bus),
                ("sabacan_robomas_reset", f"sabacan_robomas_reset_id{board_id}"),
                ("set_robomas_gains", f"set_robomas_gains_id{board_id}"),
            ],
        )

    sabacan_robomasv2_node_id1 = create_sabacan_robomasv2_node(1)
    sabacan_robomasv2_node_id2 = create_sabacan_robomasv2_node(2)
    sabacan_robomasv2_node_id3 = create_sabacan_robomasv2_node(3)
    sabacan_robomasv2_node_id4 = create_sabacan_robomasv2_node(4)
    sabacan_robomasv2_node_id5 = create_sabacan_robomasv2_node(5)
    sabacan_robomasv2_node_id6 = create_sabacan_robomasv2_node(6)
    sabacan_robomasv2_node_id7 = create_sabacan_robomasv2_node(7)
    sabacan_robomasv2_node_id8 = create_sabacan_robomasv2_node(8)

    def create_sabacan_single_control_node(
        board_id: int,
        motor_number: int,
        control_cycle: float,
        control_type: str,
        change_mode_delay: float = 0.2,
        log_level="warn",
    ) -> Node:
        return Node(
            package="sabacan_single_control",
            executable="sabacan_single_control_node",
            name=f"sabacan_single_control_node_id{board_id}_motor{motor_number}",
            parameters=[
                # param_file,
                {
                    "board_id": board_id,
                    "motor_number": motor_number,
                    "control_cycle": control_cycle,
                    "control_type": control_type,
                    "change_mode_delay": change_mode_delay,
                    "control_type_update_method": "service",
                    # control_typeの初期値はロボマスとVESCのどちらにでも対応できるようにVELOCITY
                },
            ],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=[
                ("set_robomas_gains", f"set_robomas_gains_id{board_id}"),
            ],
        )

    # 足回りは100Hz
    # vescだけ初期値はCURRENT
    sabacan_single_control_id1_motor0 = create_sabacan_single_control_node(
        1, 0, 100.0, "CURRENT", 0.0
    )
    sabacan_single_control_id1_motor1 = create_sabacan_single_control_node(
        1, 1, 100.0, "CURRENT", 0.0
    )
    sabacan_single_control_id1_motor2 = create_sabacan_single_control_node(
        1, 2, 100.0, "CURRENT", 0.0
    )
    sabacan_single_control_id1_motor3 = create_sabacan_single_control_node(
        1, 3, 100.0, "CURRENT", 0.0
    )
    sabacan_single_control_id2_motor0 = create_sabacan_single_control_node(
        2, 0, 100.0, "TORQUE", 0.0
    )
    sabacan_single_control_id2_motor1 = create_sabacan_single_control_node(
        2, 1, 100.0, "TORQUE", 0.0
    )
    sabacan_single_control_id2_motor2 = create_sabacan_single_control_node(
        2, 2, 100.0, "TORQUE", 0.0
    )
    sabacan_single_control_id2_motor3 = create_sabacan_single_control_node(
        2, 3, 100.0, "TORQUE", 0.0
    )
    # それ以外は25Hz(仮)
    sabacan_single_control_id3_motor0 = create_sabacan_single_control_node(
        3, 0, 25.0, "TORQUE", 0.0
    )
    sabacan_single_control_id3_motor1 = create_sabacan_single_control_node(
        3, 1, 25.0, "TORQUE", 0.0
    )
    sabacan_single_control_id3_motor2 = create_sabacan_single_control_node(
        3, 2, 25.0, "TORQUE", 0.0
    )
    sabacan_single_control_id3_motor3 = create_sabacan_single_control_node(
        3, 3, 25.0, "TORQUE", 0.0
    )
    sabacan_single_control_id4_motor0 = create_sabacan_single_control_node(
        4, 0, 25.0, "TORQUE", 0.0
    )
    sabacan_single_control_id4_motor1 = create_sabacan_single_control_node(
        4, 1, 25.0, "TORQUE", 0.0
    )
    sabacan_single_control_id4_motor2 = create_sabacan_single_control_node(
        4, 2, 25.0, "TORQUE", 0.0
    )
    sabacan_single_control_id4_motor3 = create_sabacan_single_control_node(
        4, 3, 25.0, "TORQUE", 0.0
    )
    sabacan_single_control_id5_motor0 = create_sabacan_single_control_node(
        5, 0, 25.0, "TORQUE", 0.0
    )
    sabacan_single_control_id5_motor1 = create_sabacan_single_control_node(
        5, 1, 25.0, "TORQUE", 0.0
    )
    sabacan_single_control_id5_motor2 = create_sabacan_single_control_node(
        5, 2, 25.0, "TORQUE", 0.0
    )
    sabacan_single_control_id5_motor3 = create_sabacan_single_control_node(
        5, 3, 25.0, "TORQUE", 0.0
    )
    sabacan_single_control_id6_motor0 = create_sabacan_single_control_node(
        6, 0, 25.0, "TORQUE", 0.0
    )
    sabacan_single_control_id6_motor1 = create_sabacan_single_control_node(
        6, 1, 25.0, "TORQUE", 0.0
    )
    sabacan_single_control_id6_motor2 = create_sabacan_single_control_node(
        6, 2, 25.0, "TORQUE", 0.0
    )
    sabacan_single_control_id6_motor3 = create_sabacan_single_control_node(
        6, 3, 25.0, "TORQUE", 0.0
    )
    # id7は計測輪のみなので不要
    sabacan_single_control_id8_motor0 = create_sabacan_single_control_node(
        8, 0, 25.0, "TORQUE", 0.0
    )
    sabacan_single_control_id8_motor1 = create_sabacan_single_control_node(
        8, 1, 25.0, "TORQUE", 0.0
    )
    sabacan_single_control_id8_motor2 = create_sabacan_single_control_node(
        8, 2, 25.0, "TORQUE", 0.0
    )
    sabacan_single_control_id8_motor3 = create_sabacan_single_control_node(
        8, 3, 25.0, "TORQUE", 0.0
    )

    def create_sabacan_gpio_node(
        board_id: int,
        from_can_bus="from_can_bus0",
        to_can_bus="to_can_bus0",
        log_level="warn",
    ) -> Node:
        return Node(
            package="sabacan",
            executable="sabacan_gpio_node",
            name=f"sabacan_gpio_node_id{board_id}",
            parameters=[param_file],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=[
                ("from_can_bus", from_can_bus),
                ("to_can_bus", to_can_bus),
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
        remappings=[
            ("from_can_bus", "from_can_bus0"),
            ("to_can_bus", "to_can_bus0"),
        ],
    )
    sabacan_led_node_id1 = Node(
        package="sabacan",
        executable="sabacan_led_node",
        name="sabacan_led_node_id1",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        remappings=[
            ("from_can_bus", "from_can_bus0"),
            ("to_can_bus", "to_can_bus0"),
        ],
    )

    def create_sabacan_robstride_node(
        board_id: int,
        from_can_bus="from_can_bus1",
        to_can_bus="to_can_bus1",
        log_level="warn",
    ) -> Node:
        return Node(
            package="sabacan",
            executable="sabacan_robstride_node",
            name=f"sabacan_robstride_node_id{board_id}",
            parameters=[param_file],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=[
                ("from_can_bus", from_can_bus),
                ("to_can_bus", to_can_bus),
            ],
        )

    # canbus1
    sabacan_robstride_node_id1 = create_sabacan_robstride_node(
        1, from_can_bus="from_can_bus1", to_can_bus="to_can_bus1"
    )

    # socket_can_bridge_launch = GroupAction(
    #     [
    #         IncludeLaunchDescription(
    #             XMLLaunchDescriptionSource(
    #                 PathJoinSubstitution(
    #                     [
    #                         FindPackageShare("ros2_socketcan"),
    #                         "launch",
    #                         "socket_can_bridge.launch.xml",
    #                     ]
    #                 )
    #             ),
    #             launch_arguments={"interface": "can0"}.items(),
    #         ),
    #     ]
    # )
    eth2can_node = Node(
        package="eth2can",
        executable="eth2can_node",
        name="eth2can_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        # remappings=[
        #     ("from_can_bus0", "from_can_bus"),
        #     ("to_can_bus0", "to_can_bus"),
        # ],
    )

    r1_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("r1_bringup"),
                "launch",
                "r1_slam.launch.py",
            )
        ),
        condition=IfCondition(use_lidar),
    )

    r1_dummy_map_node = Node(
        package="r1_control",
        executable="r1_dummy_map_node",
        name="r1_dummy_map_node",
        arguments=["--ros-args", "--log-level", "warn"],
        condition=UnlessCondition(use_lidar),
    )

    r1_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("r1_bringup"),
                "launch",
                "r1_sim.launch.py",
            )
        ),
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

    r1_aruco_display_node = Node(
        package="r1_ui",
        executable="r1_aruco_display_node",
        name="r1_aruco_display_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        condition=IfCondition(use_aruco_display),
    )

    r1_aruco_serial_node = Node(
        package="r1_ui",
        executable="r1_aruco_serial_node",
        name="r1_aruco_serial_node",
        parameters=[param_file],
        arguments=["--ros-args", "--log-level", "warn"],
        condition=IfCondition(use_aruco_display),
    )

    # r1_mainのノードの起動を遅延させる
    common_nodes = [
        # 最初にsabacanを起動
        # can0
        sabacan_robomasv2_node_id1,
        sabacan_robomasv2_node_id2,
        sabacan_robomasv2_node_id3,
        sabacan_robomasv2_node_id4,
        sabacan_robomasv2_node_id5,
        sabacan_robomasv2_node_id6,
        sabacan_robomasv2_node_id7,
        sabacan_robomasv2_node_id8,
        sabacan_gpio_node_id1,
        sabacan_gpio_node_id2,
        sabacan_gpio_node_id3,
        sabacan_power_node_id0,
        sabacan_led_node_id1,
        # can1
        sabacan_robstride_node_id1,
        # 次にsabacan_single_control以外のノードを起動
        r1_chassis_control_node,
        r1_chassis_velocity_control_node,
        r1_mecanum_node,
        # r1_swerve_drive_node,
        ps4_node,
        r1_aruco_display_node,
        r1_aruco_serial_node,
        r1_kfs_fx_node,
        r1_kfs_fz_node,
        r1_kfs_fyaw_node,
        r1_r2_flift_node,
        r1_kfs_rx_node,
        r1_kfs_rz_node,
        r1_kfs_ryaw_node,
        r1_r2_rlift_node,
        # 大槻機構
        r1_spear_y_node,
        r1_spear_roll_node,
        # 千田機構
        # r1_spear1_node,
        # r1_spear2_node,
        # r1_spear3_node,
        # r1_spear4_node,
        # r1_spear_x_node,
        # r1_spear_y_node,
        # r1_spear_roll_node,
        # r1_spear_pitch1_node,
        # r1_spear_pitch2_node,
        # 最後にsabacan_single_controlを起動
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
        # sabacan id7は計測輪のみなので不要
        sabacan_single_control_id8_motor0,
        sabacan_single_control_id8_motor1,
        sabacan_single_control_id8_motor2,
        sabacan_single_control_id8_motor3,
    ]

    real_nodes = [
        r1_slam_launch,
        r1_dummy_map_node,
        eth2can_node,
        bno086_node,
        r1_odometry_node,
    ]

    sim_nodes = [
        r1_sim_launch,
    ]

    delay_nodes = [r1_machine_manage_node, r1_main_node]

    # sabacanは遅延させて起動
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim", default_value="false"),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            DeclareLaunchArgument("use_aruco_display", default_value="false"),
            DeclareLaunchArgument(
                "robot_control_mode",
                default_value="manual",
                description="Initial mode for r1_main_node: manual or auto",
            ),
            # TimerAction(period=0.0, actions=[foxglove_node]),
            TimerAction(period=0.0, actions=common_nodes),
            TimerAction(
                period=0.0, actions=real_nodes, condition=UnlessCondition(use_sim)
            ),
            TimerAction(period=0.0, actions=sim_nodes, condition=IfCondition(use_sim)),
            TimerAction(period=3.0, actions=delay_nodes),
        ]
    )
