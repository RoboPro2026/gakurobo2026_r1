import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("r1_bringup")
    sim_param_file = os.path.join(pkg_dir, "config", "r1_sim_config.yaml")
    sim_map_yaml = "src/gakurobo2026_r1/data/map/field_blue.yaml"

    sim_map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": sim_map_yaml}],
    )

    sim_map_lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_server",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
    )

    r1_dummy_odometry_node = Node(
        package="r1_control",
        executable="r1_dummy_odometry_node",
        name="r1_dummy_odometry_node",
        parameters=[sim_param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    r1_dummy_map_node = Node(
        package="r1_control",
        executable="r1_dummy_map_node",
        name="r1_dummy_map_node",
        parameters=[sim_param_file],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    return LaunchDescription(
        [
            sim_map_server_node,
            sim_map_lifecycle_manager_node,
            r1_dummy_map_node,
            r1_dummy_odometry_node,
        ]
    )
