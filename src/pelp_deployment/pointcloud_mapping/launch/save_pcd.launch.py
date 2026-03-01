from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from datetime import datetime
from pathlib import Path

rviz_config_file = os.path.join(
    get_package_share_directory("pointcloud_mapping"),
    "rviz",
    "lbl.rviz",
)

def generate_launch_description():
    # Create the node
    pointcloud_fusion_node = Node(
        package="pointcloud_mapping",
        executable="pointcloud_fusion_node",
        name="pointcloud_fusion_node",
        output="screen",
        parameters=[
            {
                "voxel_leaf_size": LaunchConfiguration("voxel_leaf_size"),
                "map_voxel_size": LaunchConfiguration("map_voxel_size"),
                "publish_rate": LaunchConfiguration("publish_rate"),
                "max_range": LaunchConfiguration("max_range"),
                "min_range": LaunchConfiguration("min_range"),
                "save_cloud": LaunchConfiguration("save_cloud"),
                "cloud_save_path": LaunchConfiguration("cloud_save_path"),
                "snapshot_save_dir": LaunchConfiguration("snapshot_save_dir"),
            }
        ],
    )

    # Default snapshot save directory (similar to CSV path in indoor1.center.hpp_pv.launch.py)
    # Navigate up from launch file to project root, then to simulator/pointcloud_snapshots/
    default_snapshot_dir = str(
        Path(__file__)
        .resolve()
        .parents[2]
        .joinpath(
            "simulator",
            "pointcloud_snapshots",
            datetime.now().strftime("%Y%m%d_%H%M%S"),
        )
    )
    return LaunchDescription(
        [
            # Pointcloud mapping parameters (from real.lbl.sim_essential.launch.py)
            DeclareLaunchArgument("voxel_leaf_size", default_value="0.1"),
            DeclareLaunchArgument("map_voxel_size", default_value="0.1"),
            DeclareLaunchArgument("publish_rate", default_value="10.0"),
            DeclareLaunchArgument("max_range", default_value="6.0"),
            DeclareLaunchArgument("min_range", default_value="0.1"),
            DeclareLaunchArgument("cloud_save_path", default_value="/home/richard/Documents/dev_pred_exploration/src/simulator/pcd/pointcloud_map.pcd"),
            DeclareLaunchArgument("save_cloud", default_value="true"),
            DeclareLaunchArgument("snapshot_save_dir", default_value=default_snapshot_dir),
            # Node
            pointcloud_fusion_node,
            # Node(
            #     package="rviz2",
            #     executable="rviz2",
            #     name="rviz",
            #     output="log",
            #     arguments=[
            #         "-d",
            #         rviz_config_file,
            #     ],
            # ),
        ]
    )
