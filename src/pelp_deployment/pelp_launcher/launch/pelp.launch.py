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
            "map_width": "135.125",
            "map_height": "110.125",
            "map_org_x": "-5.0",  # left bottom corner as origin
            "map_org_y": "-35.5",
            "exp_map_resolution": "0.125",
            "dilation_radius": "0.125",
            "fov_deg": "360",
            "sensor_range": "15.0",
            "observed_range_limit": "10.0",
            "min_rel_z": "-0.6",
            "max_rel_z": "1.5",
        }.items(),
    )

    start_path_selector = Node(
        package="path_selector",
        executable="path_selector_fsm_node",
        name="path_selector_fsm_node",
        output="screen",
        parameters=[
            {
                "waypoint_tolerance": 1.0,
                "direction_change_penalty": 1.0,
                "odom_msg": "state_estimation",
                # "waypoint_topic": "way_point",
            }
        ],
    )

    start_path_tracker = Node(
        package="path_tracker",
        executable="path_tracker_node",
        name="path_tracker_node",
        output="screen",
        parameters=[
            {
                "odom_topic": "state_estimation",
                # "cmd_vel_topic": "cmd_vel",
                "waypoint_topic": "way_point",
                "tolerance": 0.15,
                "lookahead_distance": 0.5,
                # "max_linear_speed": 2.0,
                # "max_angular_speed": 3.0,
                # "kp_linear": 3.0,
                # "kp_angular": 1.5,
                # "kp_angular_small": 0.75,
            }
        ],
    )

    start_map_predictor = Node(
        package="grid_predictor",
        executable="grid_predictor_node",
        name="grid_predictor",
        output="screen",
        parameters=[
            {
                "robot_name": "av1",
                "model_path": "model/fpunet.pth",
                "prediction_cycle": 1.5,
            }
        ],
    )

    start_pelp_local = Node(
        package="dev_hpp_pv",
        executable="local_planner_node",
        name="local_planner",
        output="screen",
        parameters=[
            {
                "odom_topic": "state_estimation",
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
        parameters=[
            {
                "global_range": 6.0,
                "odom_topic": "state_estimation",
            }
        ],
    )

    return LaunchDescription(
        [
            start_pointcloud_mapping,
            start_path_selector,
            start_path_tracker,
            start_map_predictor,
            start_pelp_local,
            start_pelp_global,
        ]
    )
