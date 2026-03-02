#ifndef MAP_BUILDER_H
#define MAP_BUILDER_H

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <memory>
#include <mutex>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/octomap_msgs/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using PclPoint = pcl::PointXYZ;
using PclCloud = pcl::PointCloud<PclPoint>;
using PointCloudPtr = PclCloud::Ptr;

using PointCloudSubscriberPtr =
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr;
using OdomSubscriberPtr =
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr;
using OccupancyGridPublisherPtr =
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr;
using OctomapPublisherPtr =
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr;

class MapBuilder : public rclcpp::Node {
public:
  MapBuilder();

private:
  // subscribe to global pointcloud
  PointCloudSubscriberPtr global_pointcloud_subscriber;
  void globalCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  PointCloudPtr cloud;

  std::vector<std::string> vehicles;
  std::unordered_map<std::string, PointCloudSubscriberPtr>
      local_pointcloud_subscribers;
  void localCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  PointCloudPtr visible_cloud;

  // subscribe to odom
  std::unordered_map<std::string, OdomSubscriberPtr> odom_subs;
  std::unordered_map<std::string, Eigen::Vector3f> current_positions;
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg,
                    const std::string name);
  void rebuildGroundTruthFromRobotHeight(double robot_z);

  // publish ground truth and map of robot respectlvely
  nav_msgs::msg::OccupancyGrid ground_truth;
  OccupancyGridPublisherPtr ground_truth_publisher;
  nav_msgs::msg::OccupancyGrid visible;
  OccupancyGridPublisherPtr visible_publisher;

  bool map_processed;

  // thread lock
  std::mutex mutex;

  // octree
  octomap_msgs::msg::Octomap octomap_msg;
  octomap_msgs::msg::Octomap visible_octomap_msg;
  OctomapPublisherPtr ground_truth_octomap_publisher;
  OctomapPublisherPtr visible_octomap_publisher;
  std::shared_ptr<octomap::OcTree> octree_map;
  std::shared_ptr<octomap::OcTree> visible_octree_map;
  octomap::Pointcloud octo_cloud;
  octomap::Pointcloud visible_octo_cloud;
  bool ready_to_publish;
  // map params
  double resolution, origin_x, origin_y;
  int grid_size_x, grid_size_y;
  double height;
  double min_rel_z;
  double max_rel_z;
  bool ground_truth_initialized = false;
  double last_ground_truth_robot_z = 0.0;
  // params for ray simulation
  double num_rays, max_range;

  // timer
  rclcpp::TimerBase::SharedPtr timer;
  // timer callback
  void timerCallback();

  // lookup table for ray directions
  std::vector<std::pair<double, double>> ray_directions;
};

#endif