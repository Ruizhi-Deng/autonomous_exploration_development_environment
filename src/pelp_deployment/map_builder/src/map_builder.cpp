#include "../include/map_builder.h"
#include <octomap_msgs/octomap_msgs/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

MapBuilder::MapBuilder()
    : Node("map_builder"), map_processed(false), ready_to_publish(false) {
  // initialize parameter
  this->declare_parameter<double>("resolution", 1.0);
  this->declare_parameter<double>("size_x", 0.0);
  this->declare_parameter<double>("size_y", 0.0);
  this->declare_parameter<double>("height", 0.0);
  this->declare_parameter<double>("min_rel_z", -0.1);
  this->declare_parameter<double>("max_rel_z", 1.0);
  this->declare_parameter<double>("num_rays", 0.0);
  this->declare_parameter<double>("max_range", 0.0);
  resolution = this->get_parameter("resolution").as_double();
  grid_size_x = this->get_parameter("size_x").as_double();
  grid_size_y = this->get_parameter("size_y").as_double();
  height = this->get_parameter("height").as_double();
  min_rel_z = this->get_parameter("min_rel_z").as_double();
  max_rel_z = this->get_parameter("max_rel_z").as_double();
  num_rays = this->get_parameter("num_rays").as_double();
  max_range = this->get_parameter("max_range").as_double();
  origin_x = -(grid_size_x * resolution) / 2.0f;
  origin_y = -(grid_size_y * resolution) / 2.0f;
  this->declare_parameter<double>("map_org_x", origin_x);
  this->declare_parameter<double>("map_org_y", origin_y);
  origin_x = this->get_parameter("map_org_x").as_double();
  origin_y = this->get_parameter("map_org_y").as_double();

  // initialize global pointcloud subscriber
  global_pointcloud_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "overall_map", 1,
      std::bind(&MapBuilder::globalCloudCallback, this, std::placeholders::_1));

  // stores overall pointcloud
  cloud = std::make_shared<PclCloud>();

  // stores visible pointcloud (history + current)
  visible_cloud = std::make_shared<PclCloud>();

  // initialize octree
  octree_map = std::make_shared<octomap::OcTree>(resolution);
  visible_octree_map = std::make_shared<octomap::OcTree>(resolution);
  // initialize ground truth mao
  ground_truth.data.assign(grid_size_x * grid_size_y, -1);
  visible.data.assign(grid_size_x * grid_size_y, -1);

  // initialize odom and local pointcloud
  vehicles = {"av1"};
  for (auto& name : vehicles) {
    std::string local_pointcloud_topic = "registered_scan";
    std::string odom_topic = "state_estimation";
    
    odom_subs[name] = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, [this, name](nav_msgs::msg::Odometry::SharedPtr msg) {
          odomCallback(msg, name);
        });
    local_pointcloud_subscribers[name] =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            local_pointcloud_topic, 10,
            std::bind(&MapBuilder::localCloudCallback, this, std::placeholders::_1));
  }

  // create timer at 0.5 Hz for better performance
  timer = this->create_wall_timer(std::chrono::milliseconds(2000),
                                  std::bind(&MapBuilder::timerCallback, this));

  // initialize ground truth map publisher
  ground_truth_publisher =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("/ground_truth", 10);
  visible_publisher =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("/visible_map", 10);
  ground_truth_octomap_publisher =
      this->create_publisher<octomap_msgs::msg::Octomap>("/ground_truth_octomap", 10);
  visible_octomap_publisher =
      this->create_publisher<octomap_msgs::msg::Octomap>("/visible_octomap", 10);

  for (int i = 0; i < num_rays; ++i) {
    double ang = 2.0 * M_PI * i / num_rays;
    ray_directions.push_back({cos(ang), sin(ang)});
  }
}

void MapBuilder::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg,
                              const std::string name) {
  Eigen::Vector3f position(msg->pose.pose.position.x, msg->pose.pose.position.y,
                           msg->pose.pose.position.z);
  std::lock_guard<std::mutex> lock(mutex);
  current_positions[name] = position;
  // record position
  double rx = msg->pose.pose.position.x;
  double ry = msg->pose.pose.position.y;

  // if map is not processed, return
  if (!map_processed) return;

  if (!ground_truth_initialized ||
      std::abs(position.z() - last_ground_truth_robot_z) > resolution) {
    rebuildGroundTruthFromRobotHeight(position.z());
  }

  // simulate lidar with optimized ray casting
  const double step_size = 1.0 * resolution; // Larger step size for performance
  const int max_steps = static_cast<int>(max_range);

  for (int i = 0; i < num_rays; ++i) {
    double ang = 2.0 * M_PI * i / num_rays;
    double dx = cos(ang), dy = sin(ang);

    // Use Bresenham-like algorithm for faster ray traversal
    double ix = rx;
    double iy = ry;
    bool hit_obstacle = false;

    for (int step = 0; step < max_steps && !hit_obstacle; ++step) {
      int gx = static_cast<int>((ix - origin_x) / resolution);
      int gy = static_cast<int>((iy - origin_y) / resolution);

      if (gx < 0 || gx >= grid_size_x || gy < 0 || gy >= grid_size_y) break;

      int idx = gy * grid_size_x + gx;

      // set value for visible
      if (ground_truth.data[idx] == 100) {
        visible.data[idx] = 100;
        hit_obstacle = true;
      } else {
        visible.data[idx] = 0;
      }

      // Advance ray position
      ix += dx * step_size;
      iy += dy * step_size;
    }
  }

  // publish map visible to robots
  visible.header.stamp = this->get_clock()->now();
  visible.header.frame_id = "map";
  visible.info.resolution = resolution;
  visible.info.width = static_cast<uint32_t>(grid_size_x);
  visible.info.height = static_cast<uint32_t>(grid_size_y);
  visible.info.origin.position.x = origin_x;
  visible.info.origin.position.y = origin_y;
  visible.info.origin.position.z = height * resolution;
  visible.info.origin.orientation.w = 1.0; // no rotation

  visible_publisher->publish(visible);
}

void MapBuilder::rebuildGroundTruthFromRobotHeight(double robot_z) {
  std::fill(ground_truth.data.begin(), ground_truth.data.end(), -1);

  const double min_z = robot_z + min_rel_z;
  const double max_z = robot_z + max_rel_z;

  for (const auto& pt : cloud->points) {
    if (pt.z < min_z || pt.z > max_z) {
      continue;
    }

    int xIdx = static_cast<int>((pt.x - origin_x) / resolution);
    int yIdx = static_cast<int>((pt.y - origin_y) / resolution);
    if (xIdx >= 0 && xIdx < grid_size_x && yIdx >= 0 && yIdx < grid_size_y) {
      int idx = yIdx * grid_size_x + xIdx;
      ground_truth.data[idx] = 100;
    }
  }

  ground_truth_initialized = true;
  last_ground_truth_robot_z = robot_z;
}

void MapBuilder::globalCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (map_processed) {
    return;
  }
  if (!msg) {
    return;
  }

  map_processed = true;
  // initialize pointcloud
  pcl::fromROSMsg(*msg, *cloud);

  // modify octomap
  for (auto& pt : cloud->points) {
    if (pt.z < 2.0) {
      octo_cloud.push_back(pt.x, pt.y, pt.z);
    }
  }
  octomap::point3d origin_pt(0, 0, 0.0);
  octree_map->insertPointCloud(octo_cloud, origin_pt);
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree_map->getMetricMin(min_x, min_y, min_z);
  octree_map->getMetricMax(max_x, max_y, max_z);

  // generate octomap msg
  octomap_msgs::binaryMapToMsg(*octree_map, octomap_msg);

  ready_to_publish = true;
}

void MapBuilder::timerCallback() {
  if (!ready_to_publish) {
    return;
  }

  // set header
  ground_truth.header.stamp = this->get_clock()->now(); // current ROS time
  ground_truth.header.frame_id = "map";                 // or whatever frame you use

  ground_truth.info.resolution = resolution;
  ground_truth.info.width = static_cast<uint32_t>(grid_size_x);
  ground_truth.info.height = static_cast<uint32_t>(grid_size_y);
  ground_truth.info.origin.position.x = origin_x;
  ground_truth.info.origin.position.y = origin_y;
  ground_truth.info.origin.position.z = height * resolution;
  ground_truth.info.origin.orientation.w = 1.0; // no rotation

  // Publish
  ground_truth_publisher->publish(ground_truth);

  octomap_msg.header.stamp = this->get_clock()->now();
  octomap_msg.header.frame_id = "map";
  octomap_msg.resolution = octree_map->getResolution();
  ground_truth_octomap_publisher->publish(octomap_msg);

  // visible octomap
  static int last_point_count = 0;
  if ((int)visible_cloud->size() > last_point_count + 500) {
    visible_octo_cloud.clear();
    for (auto& pt : visible_cloud->points) {
      if (pt.z < 2.0) {
        visible_octo_cloud.push_back(pt.x, pt.y, pt.z);
      }
    }
    octomap::point3d origin_pt(0, 0, 0.0);
    visible_octree_map->insertPointCloud(visible_octo_cloud, origin_pt);
    double min_x, min_y, min_z, max_x, max_y, max_z;
    visible_octree_map->getMetricMin(min_x, min_y, min_z);
    visible_octree_map->getMetricMax(max_x, max_y, max_z);
    octomap_msgs::binaryMapToMsg(*visible_octree_map, visible_octomap_msg);
    visible_octomap_msg.header.stamp = this->get_clock()->now();
    visible_octomap_msg.header.frame_id = "map";
    visible_octomap_msg.resolution = visible_octree_map->getResolution();
    visible_octomap_publisher->publish(visible_octomap_msg);

    last_point_count = visible_cloud->size();
  }
}

void MapBuilder::localCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!msg) {
    return;
  }

  // convert incoming ROS message to PCL
  PointCloudPtr local_cloud = std::make_shared<PclCloud>();
  pcl::fromROSMsg(*msg, *local_cloud);

  // Only process if we have enough new points to justify downsampling
  static int point_counter = 0;
  const int downsample_threshold = 1000; // Downsample every 1000 points

  point_counter += local_cloud->size();

  // merge into global/visible cloud
  *visible_cloud += *local_cloud;

  // Only downsample when we've accumulated enough points
  if (point_counter >= downsample_threshold) {
    pcl::VoxelGrid<PclPoint> sor;
    sor.setInputCloud(visible_cloud);
    sor.setLeafSize((float)(resolution / 5.0f), (float)(resolution / 5.0f),
                    (float)(resolution / 5.0f));
    pcl::PointCloud<PclPoint>::Ptr cloud_filtered(new pcl::PointCloud<PclPoint>());
    sor.filter(*cloud_filtered);

    // update visible cloud
    *visible_cloud = *cloud_filtered;
    point_counter = 0; // Reset counter
  }

  return;
}
