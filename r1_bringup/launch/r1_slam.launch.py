import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

sys.path.append(os.path.dirname(__file__))
from lidar_reset import reset_lidar


def generate_launch_description():

    # パッケージの共有ディレクトリのパスを取得
    pkg_dir = get_package_share_directory("r1_bringup")

    # パラメータファイルのフルパスを作成
    param_file = os.path.join(pkg_dir, "config", "r1_slam_config.yaml")

    # YAMLからシリアルポートパスを読み込む
    with open(param_file, "r") as f:
        slam_params = yaml.safe_load(f)
    lidar1_port = slam_params["urg_node2_1"]["ros__parameters"]["serial_port"]
    lidar2_port = slam_params["urg_node2_2"]["ros__parameters"]["serial_port"]

    # urg_node2 起動前に LiDAR へ QT コマンド（計測停止）を送信する
    # 理由: URG が連続スキャン中のまま前セッションが終了した場合、
    #        urg_open() 内の clear_urg_communication_buffer() がデータを
    #        読み続けて無限ループに陥るため、事前に停止させる
    # reset_lidar(lidar1_port)
    # reset_lidar(lidar2_port)

    # urg_node2をライフサイクルノードとして起動
    urg_node2_1 = LifecycleNode(
        package="urg_node2",
        executable="urg_node2_node",
        name="urg_node2_1",
        remappings=[("scan", "scan1")],
        parameters=[param_file],
        namespace="",
        output="screen",
    )

    urg_node2_2 = LifecycleNode(
        package="urg_node2",
        executable="urg_node2_node",
        name="urg_node2_2",
        remappings=[("scan", "scan2")],
        parameters=[param_file],
        namespace="",
        output="screen",
    )

    # Unconfigure状態からInactive状態への遷移（auto_startがtrueのとき実施）
    urg_node2_1_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=urg_node2_1,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(urg_node2_1),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition("true"),
    )

    urg_node2_2_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=urg_node2_2,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(urg_node2_2),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition("true"),
    )

    # Inactive状態からActive状態への遷移（auto_startがtrueのとき実施）
    urg_node2_1_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=urg_node2_1,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(urg_node2_1),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition("true"),
    )

    urg_node2_2_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=urg_node2_2,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(urg_node2_2),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition("true"),
    )

    delayed_lidar_activate = TimerAction(
        period=2.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(urg_node2_1),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                ),
            ),
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(urg_node2_2),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                ),
            ),
        ],
    )

    lidar_lifecycle_watchdog = Node(
        package="r1_bringup",
        executable="lidar_lifecycle_watchdog_node",
        name="lidar_lifecycle_watchdog_node",
        output="screen",
        parameters=[
            {
                "node_names": ["urg_node2_1", "urg_node2_2"],
                "check_period": 0.2,
                "service_timeout": 0.2,
                "retry_interval": 0.4,
            }
        ],
    )

    lidar1_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.0575",
            "--y",
            "-0.0",
            "--z",
            "0.05",
            "--roll",
            "3.1415926535",
            "--pitch",
            "0.0",
            "--yaw",
            "3.1415926525",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "laser1",
        ],
    )

    lidar2_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            # xとyはうまく行かなくて実機で適当に合わせた
            "-0.04125",
            "--y",
            # "-0.427169",
            "-0.43",
            "--z",
            "0.05",
            "--roll",
            "3.1415926535",
            "--pitch",
            "0.0",
            "--yaw",
            "-1.5707963267948966",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "laser2",
        ],
    )

    # slam_toolbox = Node(
    #     package="slam_toolbox",
    #     executable="async_slam_toolbox_node",
    #     name="slam_toolbox",
    #     output="screen",
    #     parameters=[param_file],
    # )

    dual_laser_merger = Node(
        package="dual_laser_merger",
        executable="dual_laser_merger_node",
        name="dual_laser_merger",
        output="screen",
        parameters=[param_file],
    )

    # r1_laser_filter_node = Node(
    #     package="r1_control",
    #     executable="r1_laser_filter_node",
    #     name="r1_laser_filter_node",
    #     output="screen",
    #     parameters=[
    #         {
    #             "scan_topic": "/scan",
    #             "filtered_scan_topic": "/scan_filtered",
    #             "threshold": 0.8,
    #         }
    #     ],
    # )

    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": "src/gakurobo2026_r1/data/map/field_blue.yaml"}],
    )
    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[param_file],
    )
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "node_names": ["map_server", "amcl"],
            }
        ],
    )

    return LaunchDescription(
        [
            urg_node2_1,
            urg_node2_2,
            urg_node2_1_configure_event_handler,
            urg_node2_2_configure_event_handler,
            urg_node2_1_activate_event_handler,
            urg_node2_2_activate_event_handler,
            delayed_lidar_activate,
            lidar_lifecycle_watchdog,
            lidar1_tf_node,
            lidar2_tf_node,
            # slam_toolbox,
            dual_laser_merger,
            # r1_laser_filter_node,
            nav2_map_server,
            nav2_amcl,
            nav2_lifecycle_manager,
        ]
    )
