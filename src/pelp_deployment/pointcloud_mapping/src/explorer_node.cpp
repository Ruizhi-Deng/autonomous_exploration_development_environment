#include <array>
#include <cmath>
#include <deque>
#include <memory>
#include <vector>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class GridRayTracer : public rclcpp::Node {
public:
  GridRayTracer() : rclcpp::Node("grid_raytracer_node") {
    resolution_ = this->declare_parameter<double>("exp_resolution_", 0.3);
    map_size_m_x = this->declare_parameter<double>("exp_map_size_m_x", 200.0);
    map_size_m_y = this->declare_parameter<double>("exp_map_size_m_y", 200.0);
    org_exp_x = this->declare_parameter<double>("org_exp_x", 0.0);
    org_exp_y = this->declare_parameter<double>("org_exp_y", 0.0);
    sensor_range_ = this->declare_parameter<double>("sensor_range", 10.0);

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cropped_cloud", rclcpp::SensorDataQoS(),
        std::bind(&GridRayTracer::cloudCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", rclcpp::QoS(50).best_effort(),
        std::bind(&GridRayTracer::odomCallback, this, std::placeholders::_1));
    dog_pointcloud_sub_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/dog_cropped_cloud", rclcpp::SensorDataQoS(),
            std::bind(&GridRayTracer::dogCloudCallback, this,
                      std::placeholders::_1));
    sub_trimmed_dog =
        this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/stair_clusters_dog", rclcpp::QoS(1),
            std::bind(&GridRayTracer::stairDogCallback, this,
                      std::placeholders::_1));
    sub_trimmed =
        this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/stair_clusters_car", rclcpp::QoS(1),
            std::bind(&GridRayTracer::stairCarCallback, this,
                      std::placeholders::_1));
    dog_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/dog_odom", rclcpp::QoS(50).best_effort(),
        std::bind(&GridRayTracer::dogOdomCallback, this,
                  std::placeholders::_1));

    gridmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/projected_grid", 1);
    dog_gridmap_pub_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/dog_map", 1);
    raycloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/ray_endpoints", 1);
    dog_raycloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/dogray_endpoints", 1);

    width_ = static_cast<int>(map_size_m_x / resolution_);
    height_ = static_cast<int>(map_size_m_y / resolution_);

    grid_.header.frame_id = "camera_init";
    grid_.info.resolution = resolution_;
    grid_.info.width = width_;
    grid_.info.height = height_;
    grid_.info.origin.position.x = org_exp_x;
    grid_.info.origin.position.y = org_exp_y;
    grid_.info.origin.orientation.w = 1.0;
    grid_.data.assign(width_ * height_, -1);

    cleared_mask_.assign(width_ * height_, false);
    point_car_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    point_dog_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    is_locked_clear_.resize(width_ * height_, false);
    log_odds_.assign(width_ * height_, 0.0f);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    odom_buffer_.push_back(*msg);
    while (odom_buffer_.size() > 200)
      odom_buffer_.pop_front();
    pose_received_ = true;
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
  }

  void dogOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    dog_odom_buffer_.push_back(*msg);
    while (dog_odom_buffer_.size() > 200)
      dog_odom_buffer_.pop_front();
    dog_pose_received_ = true;
    dog_robot_x_ = msg->pose.pose.position.x;
    dog_robot_y_ = msg->pose.pose.position.y;
  }

  void stairDogCallback(
      const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
    for (const auto &marker : msg->markers) {
      if (marker.ns != "stair_cluster_trimmed")
        continue;

      double cx = marker.pose.position.x;
      double cy = marker.pose.position.y;
      double distance = std::hypot(cx, cy);
      if (distance > 18.0)
        continue;
      double sx = marker.scale.x;
      double sy = marker.scale.y;

      double min_x = cx - sx / 2.0;
      double max_x = cx + sx / 2.0;
      double min_y = cy - sy / 2.0;
      double max_y = cy + sy / 2.0;

      int gx_min = static_cast<int>((min_x - grid_.info.origin.position.x) /
                                    resolution_);
      int gx_max = static_cast<int>((max_x - grid_.info.origin.position.x) /
                                    resolution_);
      int gy_min = static_cast<int>((min_y - grid_.info.origin.position.y) /
                                    resolution_);
      int gy_max = static_cast<int>((max_y - grid_.info.origin.position.y) /
                                    resolution_);

      gx_min = std::max(0, gx_min);
      gy_min = std::max(0, gy_min);
      gx_max = std::min(static_cast<int>(grid_.info.width) - 1, gx_max);
      gy_max = std::min(static_cast<int>(grid_.info.height) - 1, gy_max);

      for (int gx = gx_min; gx <= gx_max; ++gx) {
        for (int gy = gy_min; gy <= gy_max; ++gy) {
          int idx = gy * grid_.info.width + gx;
          is_locked_clear_[idx] = true;
        }
      }
    }
  }

  void stairCarCallback(
      const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
    for (const auto &marker : msg->markers) {
      if (marker.ns != "stair_cluster_trimmed")
        continue;

      double cx = marker.pose.position.x;
      double cy = marker.pose.position.y;
      double distance = std::hypot(cx, cy);
      if (distance > 18.0)
        continue;
      double sx = marker.scale.x;
      double sy = marker.scale.y;

      double min_x = cx - sx / 2.0;
      double max_x = cx + sx / 2.0;
      double min_y = cy - sy / 2.0;
      double max_y = cy + sy / 2.0;

      int gx_min = static_cast<int>((min_x - grid_.info.origin.position.x) /
                                    resolution_);
      int gx_max = static_cast<int>((max_x - grid_.info.origin.position.x) /
                                    resolution_);
      int gy_min = static_cast<int>((min_y - grid_.info.origin.position.y) /
                                    resolution_);
      int gy_max = static_cast<int>((max_y - grid_.info.origin.position.y) /
                                    resolution_);

      gx_min = std::max(0, gx_min);
      gy_min = std::max(0, gy_min);
      gx_max = std::min(static_cast<int>(grid_.info.width) - 1, gx_max);
      gy_max = std::min(static_cast<int>(grid_.info.height) - 1, gy_max);

      for (int gx = gx_min; gx <= gx_max; ++gx) {
        for (int gy = gy_min; gy <= gy_max; ++gy) {
          int idx = gy * grid_.info.width + gx;
          is_locked_clear_[idx] = true;
        }
      }
    }
  }

  void
  dogCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (!dog_pose_received_)
      return;

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    double dog_x, dog_y;
    if (!getSyncedOdom(msg->header.stamp, dog_odom_buffer_, dog_x, dog_y))
      return;

    dog_robot_x_ = dog_x;
    dog_robot_y_ = dog_y;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(pcl_cloud.makeShared());
    outrem.setRadiusSearch(0.3);
    outrem.setMinNeighborsInRadius(3);
    outrem.filter(*cloud_filtered);

    insertPointsToGrid(*cloud_filtered, dog_x, dog_y);
    publishRayEndpoints(dog_x, dog_y, dog_raycloud_pub_);

    grid_.header.stamp = this->now();
    gridmap_pub_->publish(grid_);

    auto cleared_grid = grid_;
    for (size_t i = 0; i < is_locked_clear_.size(); ++i) {
      if (is_locked_clear_[i]) {
        cleared_grid.data[i] = 0;
      }
    }
    cleared_grid.header.stamp = this->now();
    dog_gridmap_pub_->publish(cleared_grid);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (!pose_received_)
      return;

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    double robot_x, robot_y;
    if (!getSyncedOdom(msg->header.stamp, odom_buffer_, robot_x, robot_y))
      return;

    robot_x_ = robot_x;
    robot_y_ = robot_y;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(pcl_cloud.makeShared());
    outrem.setRadiusSearch(0.3);
    outrem.setMinNeighborsInRadius(4);
    outrem.filter(*cloud_filtered);

    insertPointsToGrid(*cloud_filtered, robot_x_, robot_y_);
    publishRayEndpoints(robot_x, robot_y, raycloud_pub_);

    grid_.header.stamp = this->now();
    gridmap_pub_->publish(grid_);

    auto cleared_grid = grid_;
    for (size_t i = 0; i < is_locked_clear_.size(); ++i) {
      if (is_locked_clear_[i]) {
        cleared_grid.data[i] = 0;
      }
    }
    cleared_grid.header.stamp = this->now();
    dog_gridmap_pub_->publish(cleared_grid);
  }

  void insertPointsToGrid(const pcl::PointCloud<pcl::PointXYZ> &pcl_cloud,
                          double robot_x, double robot_y) {
    double origin_x = grid_.info.origin.position.x;
    double origin_y = grid_.info.origin.position.y;
    double exclusion_radius = 0.5;

    for (const auto &pt : pcl_cloud.points) {
      if ((std::abs(pt.x - robot_x_) < exclusion_radius &&
           std::abs(pt.y - robot_y_) < exclusion_radius) ||
          (std::abs(pt.x - dog_robot_x_) < exclusion_radius &&
           std::abs(pt.y - dog_robot_y_) < exclusion_radius)) {
        continue;
      }

      if (std::hypot(pt.x - robot_x, pt.y - robot_y) > sensor_range_)
        continue;

      int gx = static_cast<int>((pt.x - origin_x) / resolution_);
      int gy = static_cast<int>((pt.y - origin_y) / resolution_);
      if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_)
        continue;

      const int dxs[5] = {0, -1, 1, 0, 0};
      const int dys[5] = {0, 0, 0, -1, 1};
      for (int i = 0; i < 5; ++i) {
        int nx = gx + dxs[i];
        int ny = gy + dys[i];
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_)
          continue;
        int idx = ny * width_ + nx;
        if (cleared_mask_[idx])
          continue;
        grid_.data[idx] = 100;
      }
    }
  }

  void publishRayEndpoints(
      double robot_x, double robot_y,
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub) {
    int num_rays = 360;
    double angle_increment = 2 * M_PI / num_rays;
    pcl::PointCloud<pcl::PointXYZ> ray_endpoints;
    ray_endpoints.header.frame_id = "camera_init";

    int start_ix = static_cast<int>((robot_x - grid_.info.origin.position.x) /
                                    resolution_);
    int start_iy = static_cast<int>((robot_y - grid_.info.origin.position.y) /
                                    resolution_);

    for (int i = -42; i < 43; ++i) {
      double theta = i * angle_increment;
      double end_x = robot_x + sensor_range_ * std::cos(theta);
      double end_y = robot_y + sensor_range_ * std::sin(theta);
      int gx = static_cast<int>((end_x - grid_.info.origin.position.x) /
                                resolution_);
      int gy = static_cast<int>((end_y - grid_.info.origin.position.y) /
                                resolution_);

      ray_endpoints.points.emplace_back(end_x, end_y, 0.1);
      castRayToEndpoint(start_ix, start_iy, gx, gy);
    }

    sensor_msgs::msg::PointCloud2 ray_msg;
    pcl::toROSMsg(ray_endpoints, ray_msg);
    ray_msg.header.stamp = this->now();
    pub->publish(ray_msg);
  }

  void castRayToEndpoint(int x0, int y0, int x1, int y1) {
    int gx = x0, gy = y0;
    int dx = x1 - x0, dy = y1 - y0;
    int step_x = (dx > 0) ? 1 : -1;
    int step_y = (dy > 0) ? 1 : -1;
    double abs_dx = std::abs(dx), abs_dy = std::abs(dy);

    double tDeltaX = (dx == 0) ? 1e9 : 1.0 / abs_dx;
    double tDeltaY = (dy == 0) ? 1e9 : 1.0 / abs_dy;
    double tMaxX = 0.0, tMaxY = 0.0;

    if (dx != 0)
      tMaxX = ((step_x > 0 ? (x0 + 1) : x0) - x0) * tDeltaX;
    if (dy != 0)
      tMaxY = ((step_y > 0 ? (y0 + 1) : y0) - y0) * tDeltaY;

    int start_idx = y0 * width_ + x0;
    if (grid_.data[start_idx] != 100)
      grid_.data[start_idx] = 0;

    while (true) {
      const std::array<int, 4> dxs = {1, -1, 0, 0};
      const std::array<int, 4> dys = {0, 0, 1, -1};

      for (int k = 0; k < 4; ++k) {
        int nx = gx + dxs[k];
        int ny = gy + dys[k];
        if (nx == x1 && ny == y1)
          return;
      }

      if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_)
        return;

      int idx = gy * width_ + gx;

      if (grid_.data[idx] == 100)
        return;
      if (gx == x1 && gy == y1)
        return;

      if (tMaxX < tMaxY) {
        tMaxX += tDeltaX;
        gx += step_x;
      } else {
        tMaxY += tDeltaY;
        gy += step_y;
      }

      if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_)
        break;
      int idx2 = gy * width_ + gx;
      if (grid_.data[idx2] != 100)
        grid_.data[idx2] = 0;
    }
  }

  bool getSyncedOdom(const builtin_interfaces::msg::Time &stamp,
                     const std::deque<nav_msgs::msg::Odometry> &buffer,
                     double &x_out, double &y_out) {
    if (buffer.size() < 2)
      return false;

    double t_stamp = rclcpp::Time(stamp).seconds();
    for (size_t i = 1; i < buffer.size(); ++i) {
      const auto &prev = buffer[i - 1];
      const auto &next = buffer[i];
      double t0 = rclcpp::Time(prev.header.stamp).seconds();
      double t1 = rclcpp::Time(next.header.stamp).seconds();
      if (t0 <= t_stamp && t_stamp <= t1 && (t1 - t0) > 1e-6) {
        double ratio = (t_stamp - t0) / (t1 - t0);
        x_out = prev.pose.pose.position.x * (1 - ratio) +
                next.pose.pose.position.x * ratio;
        y_out = prev.pose.pose.position.y * (1 - ratio) +
                next.pose.pose.position.y * ratio;
        return true;
      }
    }
    return false;
  }

  std::vector<float> log_odds_;
  const float l_occ_ = 0.85f;
  const float l_free_ = -0.4f;
  const float l_min_ = -2.0f;
  const float l_max_ = 3.5f;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      dog_pointcloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dog_odom_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      sub_trimmed;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      sub_trimmed_dog;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gridmap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raycloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dog_raycloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dog_gridmap_pub_;

  std::deque<nav_msgs::msg::Odometry> odom_buffer_;
  std::deque<nav_msgs::msg::Odometry> dog_odom_buffer_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_dog_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_car_cloud_;

  nav_msgs::msg::OccupancyGrid grid_;
  std::vector<bool> cleared_mask_;
  int width_{};
  int height_{};
  double resolution_{};
  double map_size_m_x{};
  double map_size_m_y{};
  double org_exp_x{};
  double org_exp_y{};
  double sensor_range_{};
  double robot_x_{};
  double robot_y_{};
  double dog_robot_x_{};
  double dog_robot_y_{};
  bool pose_received_ = false;
  bool dog_pose_received_ = false;
  std::vector<bool> is_locked_clear_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridRayTracer>());
  rclcpp::shutdown();
  return 0;
}
