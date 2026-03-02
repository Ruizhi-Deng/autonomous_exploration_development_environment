#include "../include/path_tracker/path_tracker.h"

PathTracker::PathTracker() : Node("path_tracker_node"), odom_received(false) {
  // initialize name
  this->declare_parameter<std::string>("robot_name", "av1");
  robot = this->get_parameter("robot_name").as_string();
  
  // subscribe to odom
  declare_parameter("odom_topic", "/" + robot + "/odom");
  std::string odom_topic = this->get_parameter("odom_topic").as_string();
  odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, std::bind(&PathTracker::odomCallback, this, std::placeholders::_1));

  // subscribe to local path
  std::string path_topic = "/" + robot + "/planned_path";
  local_path_subscriber = this->create_subscription<nav_msgs::msg::Path>(
      path_topic, 10,
      std::bind(&PathTracker::localPathCallback, this, std::placeholders::_1));

  // subscribe to adjust signal
  std::string adjust_topic = "/" + robot + "/adjust";
  adjust_subscriber = this->create_subscription<std_msgs::msg::String>(
      adjust_topic, 10,
      std::bind(&PathTracker::adjustCallback, this, std::placeholders::_1));

  // publish cmd_vel
  std::string cmd_vel_topic = "/" + robot + "/cmd_vel";
  cmd_vel_publisher =
      this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

  // publish waypoint
  std::string waypoint_topic =
      this->declare_parameter<std::string>("waypoint_topic", "/" + robot + "/way_point");
  waypoint_topic = this->get_parameter("waypoint_topic").as_string();
  waypoint_publisher =
      this->create_publisher<geometry_msgs::msg::PointStamped>(waypoint_topic, 10);

  // publish target point marker
  std::string target_point_topic = "/" + robot + "/target_point";
  target_point_publisher =
      this->create_publisher<visualization_msgs::msg::Marker>(target_point_topic, 10);

  std::string all_waypoints_topic = "/" + robot + "/all_waypoints";
  all_waypoints_publisher =
      this->create_publisher<visualization_msgs::msg::Marker>(all_waypoints_topic, 10);

  // initialize parameters
  this->declare_parameter<double>("kp_linear", 1.0);
  this->declare_parameter<double>("kp_angular", 1.0);
  this->declare_parameter<double>("kp_angular_small", 0.5);
  this->declare_parameter<double>("tolerance", 0.05);
  this->declare_parameter<double>("lookahead_distance", 0.1);
  this->declare_parameter<double>("max_linear_velocity", 0.5);
  this->declare_parameter<double>("max_angular_velocity", 1.0);
  this->get_parameter("kp_linear", kp_linear);
  this->get_parameter("kp_angular", kp_angular);
  this->get_parameter("kp_angular_small", kp_angular_small);
  this->get_parameter("tolerance", tolerance);
  this->get_parameter("lookahead_distance", lookahead_distance);
  this->get_parameter("max_linear_velocity", max_linear_velocity);
  this->get_parameter("max_angular_velocity", max_angular_velocity);
}

void PathTracker::adjustCallback(const std_msgs::msg::String::SharedPtr msg) {
  if (status == msg->data) {
    return;
  }
  status = msg->data;
}

void PathTracker::localPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  last_path = path;
  path.poses.clear();
  if (msg->poses.empty() && status == "IN") {
    send_cmd_vel(0, 0);
    return;
  }
  path = *msg;

  // only reset current_index if
  // 1. it's out of bounds or
  if (current_index >= path.poses.size()) {
    current_index = 0;
    RCLCPP_INFO(this->get_logger(), "Current index out of bounds. Resetting to 0.");
    return;
  }
  // 2. if this is a completely new path
  // if (!last_path.poses.empty() && path.header.stamp != last_path.header.stamp) {
  double new_path_threshold = 0.1; // 10 cm
  if (!last_path.poses.empty() &&
      std::hypot(path.poses[0].pose.position.x - last_path.poses[0].pose.position.x,
                 path.poses[0].pose.position.y - last_path.poses[0].pose.position.y) >
          new_path_threshold) {
    current_index = 1;
    if (DEBUG_MODE)
      RCLCPP_INFO(this->get_logger(), "Path updated. Resetting current index to 1.");
    return;
  }
  // 3. current_index too far ahead of robot
  double dx = path.poses[current_index].pose.position.x - current_pose.position.x;
  double dy = path.poses[current_index].pose.position.y - current_pose.position.y;
  double dist_sq = dx * dx + dy * dy;
  if (current_index > 0 &&
      dist_sq > 9 * lookahead_distance * lookahead_distance) { // 3x lookahead distance
    // find closest point again
    double min_dist_sq = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    for (size_t i = 0; i < path.poses.size(); ++i) {
      double dx = path.poses[i].pose.position.x - current_pose.position.x;
      double dy = path.poses[i].pose.position.y - current_pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        closest_idx = i;
      }
    }
    current_index = closest_idx;
    RCLCPP_INFO(this->get_logger(),
                "Current index too far ahead of robot. Resetting to closest point.");
    return;
  }
}

void PathTracker::send_cmd_vel(double v, double w) {
  geometry_msgs::msg::Twist target_velocity;
  target_velocity.linear.x = v;
  target_velocity.angular.z = w;
  cmd_vel_publisher->publish(target_velocity);
  // RCLCPP_INFO(this->get_logger(), "Published cmd_vel: v=%.2f, w=%.2f", v, w);
}

void PathTracker::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pose = msg->pose.pose;
  odom_received = true;

  if (path.poses.empty()) {
    send_cmd_vel(0, 0);
    return;
  }

  // 1. Find the point closest to the robot
  double min_dist_sq = std::numeric_limits<double>::max();
  size_t closest_idx = 0;
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - current_pose.position.x;
    double dy = path.poses[i].pose.position.y - current_pose.position.y;
    double dist_sq = dx * dx + dy * dy;
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_idx = i;
    }
  }

  current_index = closest_idx;

  // Check if we reached the goal (and are close enough)
  if (current_index == path.poses.size() - 1) {
    if (min_dist_sq < tolerance) {
      RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping.");
      send_cmd_vel(0, 0);
      return;
    }
  }

  // 2. Lookahead logic
  size_t target_idx = current_index;
  for (size_t i = current_index; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - current_pose.position.x;
    double dy = path.poses[i].pose.position.y - current_pose.position.y;
    double dist_sq = dx * dx + dy * dy;

    target_idx = i;
    if (dist_sq >= lookahead_distance * lookahead_distance) {
      break;
    }
  }

  // RCLCPP_INFO_STREAM(this->get_logger(),
  //                    "Current index: " << current_index << ", Target index: " << target_idx
  //                                     << ", Distance to target: "
  //                                     << sqrt(pow(path.poses[target_idx].pose.position.x - current_pose.position.x, 2) +
  //                                             pow(path.poses[target_idx].pose.position.y - current_pose.position.y, 2)));

  // 2.5 Publish waypoint
  geometry_msgs::msg::PointStamped waypoint_msg;
  waypoint_msg.header.frame_id = "map";
  waypoint_msg.header.stamp = this->now();
  waypoint_msg.point = path.poses[target_idx].pose.position;
  waypoint_publisher->publish(waypoint_msg);

  // 3. Compute control command
  double dx = path.poses[target_idx].pose.position.x - current_pose.position.x;
  double dy = path.poses[target_idx].pose.position.y - current_pose.position.y;
  double distance = sqrt(dx * dx + dy * dy);

  // compute current robot orientation
  tf2::Quaternion q(current_pose.orientation.x, current_pose.orientation.y,
                    current_pose.orientation.z, current_pose.orientation.w);
  double roll, pitch, current_yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw);

  // target angle
  double target_angle = std::atan2(dy, dx);

  // error angle normalization to [-pi, pi]
  double error_angle = target_angle - current_yaw;
  while (error_angle > M_PI)
    error_angle -= 2 * M_PI;
  while (error_angle < -M_PI)
    error_angle += 2 * M_PI;

  // velocity control with better handling of large angle errors
  const double LARGE_ANGLE_THRESHOLD = M_PI / 3; // 60 degree

  double linear_vel = kp_linear * distance;
  double angular_vel = (fabs(error_angle) >= LARGE_ANGLE_THRESHOLD)
                           ? kp_angular * error_angle
                           : kp_angular_small * error_angle;

  // only stop forward motion for very large angle errors
  if (fabs(error_angle) >= LARGE_ANGLE_THRESHOLD) {
    linear_vel *= 0.05; // reduce but don't completely stop linear velocity
  }
  if (fabs(error_angle) >= M_PI / 2) {
    linear_vel = 0.0; // stop if the angle error is greater than 90 degrees
  }

  if (linear_vel > max_linear_velocity) linear_vel = max_linear_velocity;
  if (angular_vel > max_angular_velocity) angular_vel = max_angular_velocity;
  if (angular_vel < -max_angular_velocity) angular_vel = -max_angular_velocity;

  send_cmd_vel(linear_vel, angular_vel);
  publishTargetPointMarker(path.poses[target_idx].pose.position.x,
                           path.poses[target_idx].pose.position.y,
                           path.poses[target_idx].pose.position.z, {1.0, 0.0, 0.0});
  // publishTargetPointMarker(path.poses[current_index].pose.position.x,
  //  path.poses[current_index].pose.position.y,
  //  path.poses[current_index].pose.position.z,
  //  {1.0, 1.0, 0.0});
  publishAllWaypointsMarker();
}

void PathTracker::publishTargetPointMarker(double x, double y, double z,
                                           std::array<double, 3> rgb) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = robot + "/target_point";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  if (rgb.size() == 3) {
    marker.color.r = rgb[0];
    marker.color.g = rgb[1];
    marker.color.b = rgb[2];
  } else {
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  }
  target_point_publisher->publish(marker);
}

void PathTracker::publishAllWaypointsMarker() {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = robot + "/all_waypoints";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  for (const auto& pose_stamped : path.poses) {
    geometry_msgs::msg::Point p;
    p.x = pose_stamped.pose.position.x;
    p.y = pose_stamped.pose.position.y;
    p.z = pose_stamped.pose.position.z;
    marker.points.push_back(p);
  }

  all_waypoints_publisher->publish(marker);
}

void PathTracker::publishTangentLine(double idx_x, double idx_y, double idx_z,
                                     double next_x, double next_y, double next_z) {
  visualization_msgs::msg::Marker line_marker;
  line_marker.header.frame_id = "map";
  line_marker.header.stamp = this->now();
  line_marker.ns = robot + "/tangent_line";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::msg::Marker::ADD;

  geometry_msgs::msg::Point p_start;
  p_start.x = idx_x;
  p_start.y = idx_y;
  p_start.z = idx_z;

  geometry_msgs::msg::Point p_end;
  p_end.x = next_x;
  p_end.y = next_y;
  p_end.z = next_z;

  line_marker.points.clear();
  line_marker.points.push_back(p_start);
  line_marker.points.push_back(p_end);

  line_marker.scale.x = 0.02; // line width
  line_marker.color.a = 1.0;
  line_marker.color.r = 0.0;
  line_marker.color.g = 1.0;
  line_marker.color.b = 1.0;

  tangent_line_publisher->publish(line_marker);
}
