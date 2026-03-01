from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    pointcloud_mapping_launch = os.path.join(
        get_package_share_directory("pointcloud_mapping"),
        "launch",
        "mapping.launch.py",
    )

    start_pointcloud_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pointcloud_mapping_launch),
        launch_arguments={
            "map_width": "70.125",
            "map_height": "60.125",
            "map_org_x": "-5.0",  # left bottom corner as origin
            "map_org_y": "-11.5",
            "exp_map_resolution": "0.125",
            "dilation_radius": "0.3",
            "fov_deg": "360",
        }.items(),
    )

    start_path_selector = Node(
        package="path_selector",
        executable="path_selector_fsm_node",
        name="path_selector_fsm_node",
        output="screen",
        parameters=[
            {
                "waypoint_tolerance": 0.5,
                "direction_change_penalty": 1.0,
            }
        ],
    )

    start_map_predictor = Node(
        package="grid_predictor",
        executable="grid_predictor_node",
        name="grid_predictor",
        output="screen",
        parameters=[{"robot_name": "av1", "model_path": "model/fpunet.pth"}],
    )

    start_pelp_local = Node(
        package="dev_hpp_pv",
        executable="local_planner_node",
        name="local_planner",
        output="screen",
        parameters=[
            {
                "max_range_of_local_frontiers": 120.0,
                "replan_path_length_threshold": 1.0,
                "time_threshold": 1.5,
                "frontier_detection_range": -1.0,
                "global_planning_failure_threshold": 1,
                "local_cluster_size": 30,
                "global_cluster_size": 60,
                "direction_change_penalty": 0.0,
                "predicted_confidence_multiplier": 1.0,
                "turn_cost_factor": 10.0,
                "planning_cycle": 2.0,
                "a_star_search_threshold": 30.0,
            }
        ],
    )

    start_pelp_global = Node(
        package="dev_hpp_pv",
        executable="global_planner_node",
        name="global_planner",
        output="screen",
        parameters=[{"global_range": 6.0}],
    )

    return LaunchDescription(
        [
            start_pointcloud_mapping,
            # start_path_selector,
            start_map_predictor,
            start_pelp_local,
            start_pelp_global,
        ]
    )
