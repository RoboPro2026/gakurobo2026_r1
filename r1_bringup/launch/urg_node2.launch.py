# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.actions import Node


# def generate_launch_description():

#     urg_node_launch = IncludeLaunchDescription(
#         launch_description_source=PythonLaunchDescriptionSource(
#             [get_package_share_directory("urg_node2"), "/launch/urg_node2.launch.py"]
#         )
#     )

#     laser_filter_node = Node(
#         package="laser_filters",
#         executable="scan_to_scan_filter_chain",
#         parameters=[
#             PathJoinSubstitution(
#                 [
#                     get_package_share_directory("laser_filters"),
#                     "examples",
#                     "box_filter_example.yaml",
#                 ]
#             )
#         ],
#         remappings=[("/scan", "/scan"), ("/scan_filtered", "/scan_filtered")],
#     )

#     tf2_base_link_ldlidar_base = Node(
#         package="tf2_ros",
#         executable="static_transform_publisher",
#         arguments=["0", "0", "0", "0", "0", "0", "base_link", "laser"],
#     )

#     bringup_dir = get_package_share_directory("bringup")
#     cartographer_config_dir = os.path.join(bringup_dir, "config")
#     configuration_basename = "noimu_lds_2d.lua"

#     use_sim_time = False
#     resolution = "0.05"
#     publish_period_sec = "1.0"

#     cartographer_node = Node(
#         package="cartographer_ros",
#         executable="cartographer_node",
#         name="cartographer_node",
#         output="screen",
#         parameters=[{"use_sim_time": use_sim_time}],
#         arguments=[
#             "-configuration_directory",
#             cartographer_config_dir,
#             "-configuration_basename",
#             configuration_basename,
#         ],
#         remappings=[("scan", "/scan_filtered")],
#     )

#     occupancy_grid_node = Node(
#         package="cartographer_ros",
#         executable="cartographer_occupancy_grid_node",
#         name="cartographer_occupancy_grid_node",
#         output="screen",
#         parameters=[{"use_sim_time": use_sim_time}],
#         arguments=[
#             "-resolution",
#             resolution,
#             "-publish_period_sec",
#             publish_period_sec,
#         ],
#     )

#     rviz2_config = os.path.join(
#         get_package_share_directory("bringup"), "rviz", "config.rviz"
#     )

#     rviz2_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="screen",
#         arguments=[["-d"], [rviz2_config]],
#     )

#     ld = LaunchDescription()
#     ld.add_action(urg_node_launch)
#     ld.add_action(laser_filter_node)
#     ld.add_action(tf2_base_link_ldlidar_base)
#     ld.add_action(cartographer_node)
#     ld.add_action(occupancy_grid_node)
#     ld.add_action(rviz2_node)
#     return ld


# Copyright 2022 eSOL Co.,Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():

    # パラメータファイルのパス設定
    config_file_path = os.path.join(
        get_package_share_directory("r1_bringup"), "config", "params_serial.yaml"
    )

    # パラメータファイルのロード
    with open(config_file_path, "r") as file:
        config_params = yaml.safe_load(file)["urg_node2"]["ros__parameters"]

    # urg_node2をライフサイクルノードとして起動
    lifecycle_node = LifecycleNode(
        package="urg_node2",
        executable="urg_node2_node",
        name=LaunchConfiguration("node_name"),
        remappings=[("scan", LaunchConfiguration("scan_topic_name"))],
        parameters=[config_params],
        namespace="",
        output="screen",
    )

    # Unconfigure状態からInactive状態への遷移（auto_startがtrueのとき実施）
    urg_node2_node_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_start")),
    )

    # Inactive状態からActive状態への遷移（auto_startがtrueのとき実施）
    urg_node2_node_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_start")),
    )

    # パラメータについて
    # auto_start      : 起動時自動でActive状態まで遷移 (default)true
    # node_name       : ノード名 (default)"urg_node2"
    # scan_topic_name : トピック名 (default)"scan" *マルチエコー非対応*
    return LaunchDescription(
        [
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument("node_name", default_value="urg_node2"),
            DeclareLaunchArgument("scan_topic_name", default_value="scan"),
            lifecycle_node,
            urg_node2_node_configure_event_handler,
            urg_node2_node_activate_event_handler,
        ]
    )
