#include <array>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
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
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

/**
 * Grid cell state integration rule (sliding‑window, 10 frames)
 * score in [0,10]
 * - start at 5 (unknown)
 * - each frame: grid==0 -> score--, grid==100 -> score++
 * New map encoding:
 *   0‑1  : FREE (0)
 *   2‑8  : UNKNOWN (-1)
 *   9‑10 : OCCUPIED (100)
 */
class GridRayTracer : public rclcpp::Node {
public:
  GridRayTracer() : rclcpp::Node("grid_raytracer_node") {
    resolution_ = this->declare_parameter<double>("exp_resolution_", 0.3);
    map_size_m_x = this->declare_parameter<double>("exp_map_size_m_x", 200.0);
    map_size_m_y = this->declare_parameter<double>("exp_map_size_m_y", 200.0);
    org_exp_x = this->declare_parameter<double>("org_exp_x", 0.0);
    org_exp_y = this->declare_parameter<double>("org_exp_y", 0.0);
    min_rel_z_ = this->declare_parameter<double>("min_rel_z", -0.1);
    max_rel_z_ = this->declare_parameter<double>("max_rel_z", 1.0);
    outlier_radius_search_ =
      this->declare_parameter<double>("outlier_radius_search", 0.75);
    outlier_min_neighbors_ =
      this->declare_parameter<int>("outlier_min_neighbors", 3);
    sensor_range_ = this->declare_parameter<double>("sensor_range", 10.0);
    observed_range_limit_ =
      this->declare_parameter<double>("observed_range_limit", 6.0);
    dilation_radius_ = this->declare_parameter<double>("dilation_radius", 0.5);
    fov_rays_ = this->declare_parameter<int>("fov_deg", 80);

    std::string scan_topic =
      this->declare_parameter<std::string>("scan_topic", "/registered_scan");
    std::string odom_topic =
      this->declare_parameter<std::string>("odom_topic", "/state_estimation");

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        scan_topic, rclcpp::SensorDataQoS(),
        std::bind(&GridRayTracer::cloudCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::QoS(50).best_effort(),
        std::bind(&GridRayTracer::odomCallback, this, std::placeholders::_1));

    gridmap_pub_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/projected_grid", 1);
    std::string visible_grid_topic =
        this->declare_parameter<std::string>("visible_grid_topic", "/visible_map");
    visible_grid_topic = this->get_parameter("visible_grid_topic").as_string();
    fused_map_pub_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(visible_grid_topic, 1);
    raycloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/ray_endpoints", 1);

    width_ = static_cast<int>(map_size_m_x / resolution_);
    height_ = static_cast<int>(map_size_m_y / resolution_);

    grid_.header.frame_id = "map";
    grid_.info.resolution = resolution_;
    grid_.info.width = width_;
    grid_.info.height = height_;
    grid_.info.origin.position.x = org_exp_x;
    grid_.info.origin.position.y = org_exp_y;
    grid_.info.origin.orientation.w = 1.0;
    grid_.data.assign(width_ * height_, -1);

    score_grid_.assign(width_ * height_, 5);
    fused_prev_state_.assign(width_ * height_, -1);
    history_.clear();

    point_car_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    odom_buffer_.push_back(*msg);
    while (odom_buffer_.size() > 200)
      odom_buffer_.pop_front();
    pose_received_ = true;
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    robot_z_ = msg->pose.pose.position.z;

    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    robot_theta_ = yaw;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (!pose_received_) return;

    std::fill(grid_.data.begin(), grid_.data.end(), -1);

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    double robot_x, robot_y, robot_z;
    if (!getSyncedOdom(msg->header.stamp, odom_buffer_, robot_x, robot_y, robot_z)) return;
    robot_x_ = robot_x;
    robot_y_ = robot_y;
    robot_z_ = robot_z;

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_z_filtered;
    for (const auto& pt : pcl_cloud.points) {
      if (pt.z >= robot_z_ + min_rel_z_ && pt.z <= robot_z_ + max_rel_z_) {
        pcl_cloud_z_filtered.points.push_back(pt);
      }
    }
    pcl_cloud_z_filtered.width = pcl_cloud_z_filtered.points.size();
    pcl_cloud_z_filtered.height = 1;
    pcl_cloud_z_filtered.is_dense = true;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(pcl_cloud_z_filtered.makeShared());
    outrem.setRadiusSearch(outlier_radius_search_);
    outrem.setMinNeighborsInRadius(outlier_min_neighbors_);
    outrem.filter(*cloud_filtered);

    insertPointsToGrid(*cloud_filtered, robot_x_, robot_y_);
    // insertPointsToGrid(pcl_cloud, robot_x_, robot_y_);
    publishRayEndpoints(robot_x_, robot_y_, robot_theta_, raycloud_pub_);

    std::vector<bool> observed_mask_(width_ * height_, false);
    double range_limit = observed_range_limit_;

    int center_x =
        static_cast<int>((robot_x_ - grid_.info.origin.position.x) / resolution_);
    int center_y =
        static_cast<int>((robot_y_ - grid_.info.origin.position.y) / resolution_);
    int cells = static_cast<int>(range_limit / resolution_);

    for (int dx = -cells; dx <= cells; ++dx) {
      for (int dy = -cells; dy <= cells; ++dy) {
        int nx = center_x + dx;
        int ny = center_y + dy;
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

        double dist = std::hypot(dx * resolution_, dy * resolution_);
        if (dist > range_limit) continue;

        int idx = ny * width_ + nx;
        observed_mask_[idx] = true;
      }
    }

    addFrameToHistory(grid_.data, observed_mask_);

    nav_msgs::msg::OccupancyGrid fused = buildFusedGrid();

    auto stamp_now = this->now();
    grid_.header.stamp = stamp_now;
    fused.header.stamp = stamp_now;
    gridmap_pub_->publish(grid_);
    fused_map_pub_->publish(fused);
  }

  void addFrameToHistory(const std::vector<int8_t>& frame,
                         const std::vector<bool>& mask) {
    if (history_.size() == 10) {
      updateScoreGrid(history_.front().first, history_.front().second);
      history_.pop_front();
    }
    history_.emplace_back(frame, mask);
    updateScoreGrid(frame, mask);
  }

  void updateScoreGrid(const std::vector<int8_t>& frame,
                       const std::vector<bool>& observed_mask) {
    for (size_t idx = 0; idx < frame.size(); ++idx) {
      if (!observed_mask[idx]) continue;
      if (frame[idx] == 0) {
        score_grid_[idx] = std::max(0, std::min(10, score_grid_[idx] - 1));
      } else if (frame[idx] == 100) {
        score_grid_[idx] = std::max(0, std::min(10, score_grid_[idx] + 1));
      }
    }
  }

  nav_msgs::msg::OccupancyGrid buildFusedGrid() {
    nav_msgs::msg::OccupancyGrid fused = grid_;
    fused.data.resize(width_ * height_);
    for (size_t idx = 0; idx < fused.data.size(); ++idx) {
      int s = score_grid_[idx];
      if (s <= 4) {
        fused.data[idx] = 0;
      } else if (s >= 6) {
        fused.data[idx] = 100;
      } else {
        if (fused_prev_state_[idx] == -1) {
          fused.data[idx] = -1;
        } else {
          fused.data[idx] = fused_prev_state_[idx];
        }
      }
    }
    fused_prev_state_ = fused.data;
    return fused;
  }

  void insertPointsToGrid(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, double robot_x,
                          double robot_y) {
    double origin_x = grid_.info.origin.position.x;
    double origin_y = grid_.info.origin.position.y;
    int dilation_cells = static_cast<int>(std::ceil(dilation_radius_ / resolution_));

    for (const auto& pt : pcl_cloud.points) {
      if (std::hypot(pt.x - robot_x, pt.y - robot_y) > sensor_range_) continue;

      int gx = static_cast<int>((pt.x - origin_x) / resolution_);
      int gy = static_cast<int>((pt.y - origin_y) / resolution_);
      if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) continue;

      // inflation size
      for (int dx = -dilation_cells; dx <= dilation_cells; ++dx) {
        for (int dy = -dilation_cells; dy <= dilation_cells; ++dy) {
          int nx = gx + dx;
          int ny = gy + dy;
          if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

          // circle inflation
          double dist = std::hypot(dx * resolution_, dy * resolution_);
          if (dist <= dilation_radius_) {
            int idx = ny * width_ + nx;
            grid_.data[idx] = 100;
          }
        }
      }
    }
  }

  void publishRayEndpoints(
      double robot_x, double robot_y, double robot_theta,
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub) {
    int num_rays = 360;
    double angle_increment = 2 * M_PI / num_rays;
    pcl::PointCloud<pcl::PointXYZ> ray_endpoints;
    ray_endpoints.header.frame_id = "camera_init";

    int start_ix =
        static_cast<int>((robot_x - grid_.info.origin.position.x) / resolution_);
    int start_iy =
        static_cast<int>((robot_y - grid_.info.origin.position.y) / resolution_);

    for (int i = -fov_rays_ / 2; i < fov_rays_ / 2; ++i) {
      double theta = i * angle_increment;
      double end_x = robot_x + sensor_range_ * std::cos(theta + robot_theta);
      double end_y = robot_y + sensor_range_ * std::sin(theta + robot_theta);
      int gx = static_cast<int>((end_x - grid_.info.origin.position.x) / resolution_);
      int gy = static_cast<int>((end_y - grid_.info.origin.position.y) / resolution_);

      ray_endpoints.points.emplace_back(end_x, end_y, 0.1);
      castRayToEndpoint(start_ix, start_iy, gx, gy);
    }

    sensor_msgs::msg::PointCloud2 ray_msg;
    pcl::toROSMsg(ray_endpoints, ray_msg);
    ray_msg.header.stamp = this->now();
    pub->publish(ray_msg);
  }

  void castRayToEndpoint(int x0, int y0, int x1, int y1) {
    if (x0 < 0 || x0 >= width_ || y0 < 0 || y0 >= height_) return;

    int gx = x0, gy = y0;
    int dx = x1 - x0, dy = y1 - y0;
    int step_x = (dx > 0) ? 1 : -1;
    int step_y = (dy > 0) ? 1 : -1;
    double abs_dx = std::abs(dx), abs_dy = std::abs(dy);

    double tDeltaX = (dx == 0) ? 1e9 : 1.0 / abs_dx;
    double tDeltaY = (dy == 0) ? 1e9 : 1.0 / abs_dy;
    double tMaxX = 0.0, tMaxY = 0.0;

    if (dx != 0) tMaxX = ((step_x > 0 ? (x0 + 1) : x0) - x0) * tDeltaX;
    if (dy != 0) tMaxY = ((step_y > 0 ? (y0 + 1) : y0) - y0) * tDeltaY;

    int start_idx = y0 * width_ + x0;
    if (grid_.data[start_idx] != 100) grid_.data[start_idx] = 0;

    while (true) {
      const std::array<int, 4> dxs = {1, -1, 0, 0};
      const std::array<int, 4> dys = {0, 0, 1, -1};

      for (int k = 0; k < 4; ++k) {
        int nx = gx + dxs[k];
        int ny = gy + dys[k];
        if (nx == x1 && ny == y1) return;
      }

      if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) return;

      int idx = gy * width_ + gx;

      if (grid_.data[idx] == 100) return;
      if (gx == x1 && gy == y1) return;

      if (tMaxX < tMaxY) {
        tMaxX += tDeltaX;
        gx += step_x;
      } else {
        tMaxY += tDeltaY;
        gy += step_y;
      }

      if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) break;
      int idx2 = gy * width_ + gx;
      if (grid_.data[idx2] != 100) grid_.data[idx2] = 0;
    }
  }

  bool getSyncedOdom(const builtin_interfaces::msg::Time& stamp,
                     const std::deque<nav_msgs::msg::Odometry>& buffer, double& x_out,
                     double& y_out, double& z_out) {
    if (buffer.size() < 2) return false;

    double t_stamp = rclcpp::Time(stamp).seconds();
    for (size_t i = 1; i < buffer.size(); ++i) {
      const auto& prev = buffer[i - 1];
      const auto& next = buffer[i];
      double t0 = rclcpp::Time(prev.header.stamp).seconds();
      double t1 = rclcpp::Time(next.header.stamp).seconds();
      if (t0 <= t_stamp && t_stamp <= t1 && (t1 - t0) > 1e-6) {
        double ratio = (t_stamp - t0) / (t1 - t0);
        x_out =
            prev.pose.pose.position.x * (1 - ratio) + next.pose.pose.position.x * ratio;
        y_out =
            prev.pose.pose.position.y * (1 - ratio) + next.pose.pose.position.y * ratio;
        z_out =
            prev.pose.pose.position.z * (1 - ratio) + next.pose.pose.position.z * ratio;
        return true;
      }
    }
    return false;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gridmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr fused_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raycloud_pub_;

  std::vector<int8_t> fused_prev_state_;
  std::deque<nav_msgs::msg::Odometry> odom_buffer_;
  std::deque<std::pair<std::vector<int8_t>, std::vector<bool>>> history_;

  nav_msgs::msg::OccupancyGrid grid_;
  std::vector<int> score_grid_;

  int width_{};
  int height_{};
  double resolution_{};
  double map_size_m_x{};
  double map_size_m_y{};
  double org_exp_x{};
  double org_exp_y{};
  double min_rel_z_{};
  double max_rel_z_{};
  double outlier_radius_search_{};
  int outlier_min_neighbors_{};
  double sensor_range_{};
  double observed_range_limit_{};
  double robot_x_{};
  double robot_y_{};
  double robot_z_{};
  double robot_theta_{};
  double dilation_radius_{};
  int fov_rays_{};
  bool pose_received_ = false;

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_car_cloud_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridRayTracer>());
  rclcpp::shutdown();
  return 0;
}
