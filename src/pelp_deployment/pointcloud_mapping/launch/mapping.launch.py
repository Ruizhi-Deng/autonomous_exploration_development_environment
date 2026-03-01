from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

rviz_config_file = os.path.join(
    get_package_share_directory("pointcloud_mapping"),
    "rviz",
    "lbl.rviz",
)

def generate_launch_description():
    map_width = LaunchConfiguration("map_width")
    map_height = LaunchConfiguration("map_height")
    map_org_x = LaunchConfiguration("map_org_x")
    map_org_y = LaunchConfiguration("map_org_y")
    exp_map_resolution = LaunchConfiguration("exp_map_resolution")
    dilation_radius = LaunchConfiguration("dilation_radius")
    fov_deg = LaunchConfiguration("fov_deg")
    
    # PointCloud Mapper param
    voxel_leaf_size = LaunchConfiguration("voxel_leaf_size")
    map_voxel_size = LaunchConfiguration("map_voxel_size")
    publish_rate = LaunchConfiguration("publish_rate")
    max_range = LaunchConfiguration("max_range")
    min_range = LaunchConfiguration("min_range")
    max_height = LaunchConfiguration("max_height")
    pcd_save_path = LaunchConfiguration("cloud_save_path")
    save_cloud_ = LaunchConfiguration("save_cloud")
    cloud_topic = LaunchConfiguration("cloud_topic")
    odom_topic = LaunchConfiguration("odom_topic")



    prob_grid_node_node = Node(
        package="pointcloud_mapping",
        executable="prob_grid_node",
        name="probability_grid_node",
        output="screen",
        parameters=[
            {
                "exp_resolution_": exp_map_resolution,
                "exp_map_size_m_x": map_width,
                "exp_map_size_m_y": map_height,
                "org_exp_x": map_org_x,
                "org_exp_y": map_org_y,
                "sensor_range": 10.0,
                "dilation_radius": dilation_radius,
                "fov_deg": fov_deg,
            }
        ],
    )
    
    pointcloud_fusion_node = Node(
        package="pointcloud_mapping",
        executable="pointcloud_fusion_node",
        name="pointcloud_fusion_node",
        output="screen",
        parameters=[
            {
                "voxel_leaf_size": voxel_leaf_size,
                "map_voxel_size": map_voxel_size,
                "publish_rate": publish_rate,
                "max_range": max_range,
                "min_range": min_range,
                "max_height": max_height,
                "cloud_save_path": pcd_save_path,
                "save_cloud": save_cloud_,
                "cloud_topic": cloud_topic,
                "odom_topic": odom_topic,
            }
        ],
    )

    map_saver_node = Node(
        package="pointcloud_mapping",
        executable="map_saver_node",
        name="map_saver_node",
        output="screen",
    )

    rviz_node = Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                output="log",
                arguments=[
                    "-d",
                    rviz_config_file,
                ],
    )

    return LaunchDescription(
        [
            # param for probabilistic grid
            DeclareLaunchArgument("map_width", default_value="50.0"),
            DeclareLaunchArgument("map_height", default_value="50.0"),
            DeclareLaunchArgument("map_org_x", default_value="-5.0"),
            DeclareLaunchArgument("map_org_y", default_value="-45.0"),
            DeclareLaunchArgument("exp_map_resolution", default_value="0.1"),
            DeclareLaunchArgument("dilation_radius", default_value="0.2"),
            DeclareLaunchArgument("fov_deg", default_value="360"),
            # pointcloud mapping
            DeclareLaunchArgument("voxel_leaf_size", default_value="0.01"),
            DeclareLaunchArgument("map_voxel_size", default_value="0.05"),
            DeclareLaunchArgument("publish_rate", default_value="10.0"),
            DeclareLaunchArgument("max_range", default_value="6.0"),
            DeclareLaunchArgument("min_range", default_value="0.1"),
            DeclareLaunchArgument("max_height", default_value="2.0"),
            DeclareLaunchArgument("cloud_save_path", default_value="/tmp/pointcloud_map.pcd"),
            DeclareLaunchArgument("save_cloud", default_value="false"),
            DeclareLaunchArgument("cloud_topic", default_value="/av1/local_pointcloud"),
            DeclareLaunchArgument("odom_topic", default_value="/av1/odom"),
            
            # node
            prob_grid_node_node,
            # pointcloud_fusion_node,
            # map_saver_node,
            # rviz_node,
        ]
    )
