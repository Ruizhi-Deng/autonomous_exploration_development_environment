#ifndef PATH_TRACKER_H
#define PATH_TRACKER_H

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>

#define DEBUG_MODE false

using OdomSubscriberPtr = rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr;
using PathSubscriberPtr = rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr;
using StringSubscriberPtr = rclcpp::Subscription<std_msgs::msg::String>::SharedPtr;
using TwistPublisherPtr = rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr;
using MarkerPublisherPtr = rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr;
using WaypointPublisherPtr =
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr;

class PathTracker : public rclcpp::Node {
public:
  PathTracker();

private:
  // manage robot
  std::string robot;

  // subscribe to odom
  geometry_msgs::msg::Pose current_pose;
  OdomSubscriberPtr odom_subscriber;
  bool odom_received;
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // subscribe to local path
  PathSubscriberPtr local_path_subscriber;
  nav_msgs::msg::Path path, last_path;
  size_t current_index = 0;
  size_t last_target_index = 0;
  void localPathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  // subscribe to adjust signal
  StringSubscriberPtr adjust_subscriber;
  std::string status = "IN";
  void adjustCallback(const std_msgs::msg::String::SharedPtr msg);

  // publish twist (cmd_vel)
  TwistPublisherPtr cmd_vel_publisher;
  void send_cmd_vel(double v, double w);

  // publish waypoint
  WaypointPublisherPtr waypoint_publisher;

  // trakcing parameter, we apply a P controller
  double kp_linear;        // P for linear
  double kp_angular;       // P for angular
  double kp_angular_small; // P when angular is small
  double tolerance;
  double lookahead_distance;
  double max_linear_velocity, max_angular_velocity; // dynamic limit

  // publish marker for goal point
  MarkerPublisherPtr target_point_publisher, all_waypoints_publisher,
      tangent_line_publisher;
  void publishTargetPointMarker(double x, double y, double z, std::array<double, 3> rgb);
  void publishAllWaypointsMarker();
  void publishTangentLine(double idx_x, double idx_y, double idx_z, double next_x,
                          double next_y, double next_z);
};

#endif