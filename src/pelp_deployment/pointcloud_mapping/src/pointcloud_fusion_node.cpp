#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <deque>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sstream>
#include <std_msgs/msg/bool.hpp>

// PointCloudMapper
// construct global pointcloud map
class PointCloudMapper : public rclcpp::Node {
 public:
  PointCloudMapper() : rclcpp::Node("pointcloud_mapper_node") {
    voxel_leaf_size_ = this->declare_parameter<double>("voxel_leaf_size", 0.2);
    // downsample rate, unit: m
    map_voxel_size_ = this->declare_parameter<double>("map_voxel_size", 0.2);
    publish_rate_ = this->declare_parameter<double>("publish_rate", 10.0);
    max_range_ = this->declare_parameter<double>("max_range", 50.0);
    min_range_ = this->declare_parameter<double>("min_range", 0.5);
    max_height_ = this->declare_parameter<double>("max_height", 100.0);
    save_cloud_ = this->declare_parameter<bool>("save_cloud", true);
    cloud_save_path_ = this->declare_parameter<std::string>(
        "cloud_save_path", "/tmp/pointcloud_map.pcd");
    snapshot_save_dir_ = this->declare_parameter<std::string>(
        "snapshot_save_dir", "/tmp/pointcloud_snapshots");
    std::string cloud_topic = this->declare_parameter<std::string>(
        "cloud_topic", "/cloud_registered");
    std::string odom_topic = this->declare_parameter<std::string>(
        "odom_topic", "/av1/lidar_odom");
    save_count_ = 0;

    RCLCPP_INFO(this->get_logger(),
                "PointCloudMapper initialized with voxel_leaf_size=%.2f, "
                "map_voxel_size=%.2f",
                voxel_leaf_size_, map_voxel_size_);

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, rclcpp::SensorDataQoS(),
        std::bind(&PointCloudMapper::cloudCallback, this,
                  std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::QoS(50).best_effort(),
        std::bind(&PointCloudMapper::odomCallback, this,
                  std::placeholders::_1));

    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/exp_pointcloud", 1);

    cropped_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/exp_pointcloud_cropped", 1);

    registered_cropped_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered_cropped", 1);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
        std::bind(&PointCloudMapper::publishMap, this));

    save_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/save", 10,
        std::bind(&PointCloudMapper::saveCallback, this,
                  std::placeholders::_1));

    global_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    has_odom_ = false;

    RCLCPP_INFO(this->get_logger(),
                "PointCloudMapper node started, waiting for data...");
  }

  ~PointCloudMapper() {
    // save pointcloud when exit
    if (save_cloud_) {
      saveCloudToFile();
    }
  }

 private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = *msg;
    has_odom_ = true;
  }

  void saveCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      saveCloudWithTimestamp();
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!has_odom_) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *raw_cloud);

    if (raw_cloud->empty()) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(raw_cloud);
    voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_,
                             voxel_leaf_size_);
    voxel_filter.filter(*cloud);

    if (cloud->empty()) {
      return;
    }

    if (registered_cropped_pub_->get_subscription_count() > 0) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_cloud(
          new pcl::PointCloud<pcl::PointXYZI>());
      for (const auto& point : cloud->points) {
        if (point.z <= max_height_) {
          cropped_cloud->push_back(point);
        }
      }
      sensor_msgs::msg::PointCloud2 cropped_output_msg;
      pcl::toROSMsg(*cropped_cloud, cropped_output_msg);
      cropped_output_msg.header = msg->header;
      registered_cropped_pub_->publish(cropped_output_msg);
    }

    {
      std::lock_guard<std::mutex> lock(map_mutex_);

      for (const auto& point : cloud->points) {
        // check invalid point
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
          continue;
        }

        // filter by distance
        float range =
            std::sqrt(pow(point.x - latest_odom_.pose.pose.position.x, 2) +
                      pow(point.y - latest_odom_.pose.pose.position.y, 2) +
                      pow(point.z - latest_odom_.pose.pose.position.z, 2));
        if (range < min_range_ || range > max_range_) {
          continue;
        }

        global_map_->push_back(point);
      }

      // downsample
      if (global_map_->size() > 500000) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_map(
            new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(global_map_);
        voxel_filter.setLeafSize(map_voxel_size_, map_voxel_size_,
                                 map_voxel_size_);
        voxel_filter.filter(*temp_map);
        global_map_ = temp_map;
      }
    }
  }

  void publishMap() {
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (global_map_->empty()) {
      return;
    }

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*global_map_, output_msg);
    output_msg.header.frame_id = "map";
    output_msg.header.stamp = this->now();

    map_pub_->publish(output_msg);

    // cropped map
    pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_map(
        new pcl::PointCloud<pcl::PointXYZI>());
    for (const auto& point : global_map_->points) {
      if (point.z <= max_height_) {
        cropped_map->push_back(point);
      }
    }
    sensor_msgs::msg::PointCloud2 cropped_output_msg;
    pcl::toROSMsg(*cropped_map, cropped_output_msg);
    cropped_output_msg.header.frame_id = "map";
    cropped_output_msg.header.stamp = output_msg.header.stamp;
    cropped_map_pub_->publish(cropped_output_msg);

    RCLCPP_DEBUG(this->get_logger(), "Published map with %zu points",
                 global_map_->size());
  }

  void saveCloudToFile() {
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (global_map_->empty()) {
      RCLCPP_WARN(this->get_logger(), "Global map is empty, nothing to save");
      return;
    }

    std::filesystem::path file_path(cloud_save_path_);
    std::filesystem::path dir_path = file_path.parent_path();

    if (!dir_path.empty() && !std::filesystem::exists(dir_path)) {
      try {
        std::filesystem::create_directories(dir_path);
        RCLCPP_INFO(this->get_logger(), "Created directory: %s",
                    dir_path.string().c_str());
      } catch (const std::filesystem::filesystem_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create directory %s: %s",
                     dir_path.string().c_str(), e.what());
        return;
      }
    }

    int result = pcl::io::savePCDFileBinary(cloud_save_path_, *global_map_);
    if (result == 0) {
      RCLCPP_INFO(this->get_logger(), "Pointcloud saved to %s with %zu points",
                  cloud_save_path_.c_str(), global_map_->size());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save pointcloud to %s",
                   cloud_save_path_.c_str());
    }
  }

  void saveCloudWithTimestamp() {
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (global_map_->empty()) {
      RCLCPP_WARN(this->get_logger(), "Global map is empty, nothing to save");
      return;
    }

    // Generate filename with timestamp
    std::filesystem::path dir_path(snapshot_save_dir_);

    if (!std::filesystem::exists(dir_path)) {
      try {
        std::filesystem::create_directories(dir_path);
        RCLCPP_INFO(this->get_logger(), "Created directory: %s",
                    dir_path.string().c_str());
      } catch (const std::filesystem::filesystem_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create directory %s: %s",
                     dir_path.string().c_str(), e.what());
        return;
      }
    }

    // Get current time as string
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::stringstream time_str;
    time_str << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");

    std::string filename = snapshot_save_dir_ + "/cloud_" + time_str.str() +
                           "_" + std::to_string(save_count_++) + ".pcd";

    int result = pcl::io::savePCDFileBinary(filename, *global_map_);
    if (result == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "Pointcloud snapshot saved to %s with %zu points",
                  filename.c_str(), global_map_->size());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save pointcloud to %s",
                   filename.c_str());
    }
  }

  // subscriber and publisher
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr save_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_cropped_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // param
  double voxel_leaf_size_;         // point downsample size
  double map_voxel_size_;          // map downsample size
  double publish_rate_;            // publish frequency
  double max_range_;               // max distance
  double min_range_;               // min distance
  double max_height_;              // max height to keep
  std::string cloud_save_path_;    // path to save pointcloud
  std::string snapshot_save_dir_;  // directory to save snapshots
  bool save_cloud_;                // whether to save

  // data
  nav_msgs::msg::Odometry latest_odom_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_;
  bool has_odom_;
  int save_count_;  // counter for saved snapshots

  // thread safety
  std::mutex odom_mutex_;
  std::mutex map_mutex_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudMapper>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
