#include "../include/path_selector_fsm.h"

#include <limits>

const std::vector<std::pair<std::pair<int, int>, double>> PathSelector::neighbor_dirs = {
    {{0, 1}, M_PI / 2},       // (0, 1) - up
    {{1, 0}, 0.0},            // (1, 0) - right
    {{0, -1}, -M_PI / 2},     // (0, -1) - down
    {{-1, 0}, M_PI},          // (-1, 0) - left
    {{1, 1}, M_PI / 4},       // (1, 1) - up-right diagonal
    {{-1, 1}, 3 * M_PI / 4},  // (-1, 1) - up-left diagonal
    {{1, -1}, -M_PI / 4},     // (1, -1) - down-right diagonal
    {{-1, -1}, -3 * M_PI / 4} // (-1, -1) - down-left diagonal
};

PathSelector::PathSelector()
    : Node("path_selector"), odom_received(false), predicted_received(false),
      visible_received(false), curr_index(1) {
  // get robot name from parameter
  std::string robot = this->declare_parameter<std::string>("robot_name", "av1");
  robot = this->get_parameter("robot_name").as_string();

  // subscribe to local plan result
  std::string local_topic = "/local_planned_path";
  local_plan_sub = this->create_subscription<prob_msgs::msg::PathPoseArray>(
      local_topic, 10,
      std::bind(&PathSelector::localPathCallback, this, std::placeholders::_1));

  // subscribe to global plan result
  std::string global_topic = "/global_planned_path";
  global_plan_sub = this->create_subscription<prob_msgs::msg::PathPoseArray>(
      global_topic, 10,
      std::bind(&PathSelector::globalPathCallback, this, std::placeholders::_1));

  // subscribe to odom
  this->declare_parameter<std::string>("odom_msg", "/av1/odom");
  std::string odom_topic = this->get_parameter("odom_msg").as_string();
  odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&PathSelector::odomCallback, this, std::placeholders::_1));

  // publish replan request
  replan_requestor = this->create_publisher<std_msgs::msg::Bool>("/replan_request", 10);

  // subscribe to use_local and use_global topics
  use_local_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/use_local", 10,
      std::bind(&PathSelector::useLocalCallback, this, std::placeholders::_1));
  use_global_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/use_global", 10,
      std::bind(&PathSelector::useGlobalCallback, this, std::placeholders::_1));

  // subscribe to map
  predicted_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/predicted_map", 10,
      std::bind(&PathSelector::predictedCallback, this, std::placeholders::_1));
  visible_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/visible_map", 10,
      std::bind(&PathSelector::visibleCallback, this, std::placeholders::_1));

  // publish path
  std::string path_topic = "/" + robot + "/planned_path";
  path_pub = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);
  solution_pub =
      this->create_publisher<nav_msgs::msg::Path>("/" + robot + "/solution_path", 10);

  // publish waypoint
  // std::string waypoint_topic =
  //     declare_parameter<std::string>("waypoint_topic", "/" + robot + "/way_point");
  // waypoint_topic = this->get_parameter("waypoint_topic").as_string();
  // waypoint_topic = this->get_parameter("waypoint_topic").as_string();
  // waypoint_pub =
  //     this->create_publisher<geometry_msgs::msg::PointStamped>(waypoint_topic, 10);

  // this->declare_parameter<double>("height", 0.0);
  // height = this->get_parameter("height").as_double();

  this->declare_parameter<double>("waypoint_tolerance", 0.5);
  waypoint_tolerance = this->get_parameter("waypoint_tolerance").as_double();

  this->declare_parameter<double>("direction_change_penalty", 1.0);
  direction_change_penalty = this->get_parameter("direction_change_penalty").as_double();
}

void PathSelector::localPathCallback(const prob_msgs::msg::PathPoseArray::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received new local path with %zu poses",
              msg->poses.size());
  local_res = *msg;
}

void PathSelector::globalPathCallback(
    const prob_msgs::msg::PathPoseArray::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received new global path with %zu poses",
              msg->poses.size());
  global_res = *msg;
}

void PathSelector::updateOnce() {
  // print status
  if (DEBUG_MODE)
    RCLCPP_INFO(this->get_logger(), "Current state: %s",
                selector_states[static_cast<int>(current_state)].c_str());

  SelectorState next_state;
  switch (current_state) {
    case SelectorState::READY: {
      exe_path.poses.clear();
      // Use use_local and use_global flags to select path
      if (use_local && !local_res.poses.empty()) {
        next_state = SelectorState::LOCAL_INIT;
        if (DEBUG_MODE) {
          RCLCPP_INFO(this->get_logger(),
                      "Transition to LOCAL_INIT state (use_local=true)");
        }
      } else if (use_global && !global_res.poses.empty()) {
        next_state = SelectorState::GLOBAL_INIT;
        if (DEBUG_MODE) {
          RCLCPP_INFO(this->get_logger(),
                      "Transition to GLOBAL_INIT state (use_global=true)");
        }
      } else if (!local_res.poses.empty()) {
        // Fallback to local if use_local is true
        next_state = SelectorState::LOCAL_INIT;
        if (DEBUG_MODE) {
          RCLCPP_INFO(this->get_logger(), "Transition to LOCAL_INIT state (fallback)");
        }
      } else if (!global_res.poses.empty()) {
        // Fallback to global if use_global is true
        next_state = SelectorState::GLOBAL_INIT;
        if (DEBUG_MODE) {
          RCLCPP_INFO(this->get_logger(), "Transition to GLOBAL_INIT state (fallback)");
        }
      } else {
        next_state = SelectorState::READY;
        if (DEBUG_MODE) {
          RCLCPP_INFO(this->get_logger(), "Staying in READY state, no paths available");
        }
      }
      break;
    }

    case SelectorState::LOCAL_INIT: {
      exe_path = local_res;
      curr_index = 1;
      local_res.poses.clear();
      next_state = SelectorState::LOCAL_EXEC;
      if (DEBUG_MODE) {
        RCLCPP_INFO(this->get_logger(), "Transition to LOCAL_EXEC state");
      }
      break;
    }

    case SelectorState::LOCAL_EXEC: {
      if (replan_requested) {
        next_state = SelectorState::READY;
        replan_requested = false;
        if (DEBUG_MODE)
          RCLCPP_INFO(this->get_logger(),
                      "Replan requested, transitioning to READY state");
      } else if (use_global && !global_res.poses.empty()) {
        // Switch to global path when use_global is true
        next_state = SelectorState::GLOBAL_INIT;
        if (DEBUG_MODE)
          RCLCPP_INFO(this->get_logger(),
                      "use_global=true, transitioning to GLOBAL_INIT state");
      } else if (!local_res.poses.empty()) {
        next_state = SelectorState::LOCAL_INIT;
        if (DEBUG_MODE)
          RCLCPP_INFO(this->get_logger(),
                      "New local path received, transitioning to LOCAL_INIT state");
      } else if (curr_index >= static_cast<int>(exe_path.poses.size())) {
        next_state = SelectorState::READY;
        if (DEBUG_MODE)
          RCLCPP_INFO(this->get_logger(),
                      "Completed local path, transitioning to READY state");
      } else {
        updateCurrentIndex();
        executeExePathOnce();
        next_state = SelectorState::LOCAL_EXEC;
      }
      break;
    }

    case SelectorState::GLOBAL_INIT: {
      exe_path = global_res;
      curr_index = 1;
      global_res.poses.clear();
      next_state = SelectorState::GLOBAL_EXEC;
      if (DEBUG_MODE) RCLCPP_INFO(this->get_logger(), "Transition to GLOBAL_EXEC state");
      break;
    }

    case SelectorState::GLOBAL_EXEC: {
      if (replan_requested) {
        next_state = SelectorState::READY;
        replan_requested = false;
        if (DEBUG_MODE)
          RCLCPP_INFO(this->get_logger(),
                      "Replan requested, transitioning to READY state");
      } else if (use_local && !local_res.poses.empty()) {
        // Switch to local path when use_local is true
        next_state = SelectorState::LOCAL_INIT;
        if (DEBUG_MODE)
          RCLCPP_INFO(this->get_logger(),
                      "use_local=true, transitioning to LOCAL_INIT state");
      } else if (!global_res.poses.empty()) {
        next_state = SelectorState::GLOBAL_INIT;
        if (DEBUG_MODE)
          RCLCPP_INFO(this->get_logger(),
                      "New global path received, transitioning to GLOBAL_INIT state");
      } else if (curr_index >= static_cast<int>(exe_path.poses.size())) {
        next_state = SelectorState::READY;
        if (DEBUG_MODE)
          RCLCPP_INFO(this->get_logger(),
                      "Completed global path, transitioning to READY state");
      } else {
        updateCurrentIndex();
        executeExePathOnce();
        next_state = SelectorState::GLOBAL_EXEC;
      }
      break;
    }

    default:
      break;
  }
  current_state = next_state;
}

void PathSelector::updateCurrentIndex() {
  if (curr_index < 0 || curr_index >= static_cast<int>(exe_path.poses.size())) {
    if (DEBUG_MODE)
      RCLCPP_WARN(this->get_logger(), "Invalid curr_index %d / %d, resetting to 1",
                  curr_index, static_cast<int>(exe_path.poses.size()));
    curr_index = std::min(1, static_cast<int>(exe_path.poses.size()) - 1);
  }

  // update curr_index based on distance to current waypoint
  while (curr_index < static_cast<int>(exe_path.poses.size())) {
    const auto& current_waypoint = exe_path.poses[curr_index].pose;

    // calculate distance from robot to current waypoint
    double dx = current_pose.position.x - current_waypoint.position.x;
    double dy = current_pose.position.y - current_waypoint.position.y;
    double distance_to_waypoint = std::hypot(dx, dy);

    // check if robot is close enough to current waypoint
    if (distance_to_waypoint <= waypoint_tolerance) {
      // robot has reached current waypoint, advance to next one
      curr_index++;
      DEBUG_LOG("Reached waypoint %d, advancing to waypoint %d (distance: %.3f)",
                curr_index - 1, curr_index, distance_to_waypoint);

      // log when advancing to next waypoint
      if (curr_index < static_cast<int>(exe_path.poses.size())) {
        const auto& next_waypoint = exe_path.poses[curr_index].pose;
        if (DEBUG_MODE)
          RCLCPP_INFO(this->get_logger(), "Head for waypoint %d, location (%.2f, %.2f)",
                      curr_index, next_waypoint.position.x, next_waypoint.position.y);
      }

      // check if we've reached the end of the path
      if (curr_index >= static_cast<int>(exe_path.poses.size())) {
        if (DEBUG_MODE) DEBUG_LOG("Reached final waypoint of the path");
        return;
      }
    } else {
      // robot is not close enough, stop advancing
      break;
    }
  }
}

void PathSelector::executeExePathOnce() {
  // get current target pose
  const auto& target_pose = exe_path.poses[curr_index].pose;

  // convert robot position to grid coordinates (continuous, no snapping)
  double robot_x_cont = current_pose.position.x;
  double robot_y_cont = current_pose.position.y;
  int robot_x_original = static_cast<int>(
      (robot_x_cont - predicted.info.origin.position.x) / predicted.info.resolution);
  int robot_y_original = static_cast<int>(
      (robot_y_cont - predicted.info.origin.position.y) / predicted.info.resolution);
  int robot_x = robot_x_original;
  int robot_y = robot_y_original;

  // convert target position to grid coordinates (snapped to grid)
  double snapped_goal_x = std::round(target_pose.position.x / predicted.info.resolution) *
                          predicted.info.resolution;
  double snapped_goal_y = std::round(target_pose.position.y / predicted.info.resolution) *
                          predicted.info.resolution;
  int goal_x = static_cast<int>((snapped_goal_x - predicted.info.origin.position.x) /
                                predicted.info.resolution);
  int goal_y = static_cast<int>((snapped_goal_y - predicted.info.origin.position.y) /
                                predicted.info.resolution);

  // validate robot position - if in obstacle, search nearby for free position
  if (robot_x >= 0 && robot_x < static_cast<int>(predicted.info.width) && robot_y >= 0 &&
      robot_y < static_cast<int>(predicted.info.height)) {
    int robot_idx = robot_y * predicted.info.width + robot_x;
    if (visible.data[robot_idx] >= 80) {
      // search for nearby free position
      bool found_free = false;
      for (int radius = 1; radius <= 3 && !found_free; radius++) {
        for (int dx = -radius; dx <= radius && !found_free; dx++) {
          for (int dy = -radius; dy <= radius && !found_free; dy++) {
            int nx = robot_x + dx;
            int ny = robot_y + dy;
            if (nx >= 0 && nx < static_cast<int>(predicted.info.width) && ny >= 0 &&
                ny < static_cast<int>(predicted.info.height)) {
              int nidx = ny * predicted.info.width + nx;
              if (predicted.data[nidx] < 50) {
                robot_x = nx;
                robot_y = ny;
                found_free = true;
              }
            }
          }
        }
      }
      if (!found_free) {
        RCLCPP_WARN(this->get_logger(),
                    "Robot is in obstacle and couldn't find free position nearby");
        return;
      }
    }
  }

  nav_msgs::msg::Path solution_path = convertPathPoseArrayToPath(exe_path);
  solution_pub->publish(solution_path);

  // perform A* search from robot to target
  std::vector<Point2D> a_star_path;
  if (aStarSearch(robot_x, robot_y, goal_x, goal_y, a_star_path)) {
    // convert A* path to nav_msgs::msg::Path and publish
    nav_msgs::msg::Path output_path;
    output_path.header.stamp = this->get_clock()->now();
    output_path.header.frame_id = "map";

    // add robot's current position as first point
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.header = output_path.header;
    robot_pose.pose = current_pose;
    // output_path.poses.push_back(robot_pose);

    // add A* path points
    for (size_t i = 0; i < a_star_path.size(); i++) {
      geometry_msgs::msg::PoseStamped path_pose;
      path_pose.header = output_path.header;
      path_pose.pose.position.x = predicted.info.origin.position.x +
                                  (a_star_path[i].x + 0.5) * predicted.info.resolution;
      path_pose.pose.position.y = predicted.info.origin.position.y +
                                  (a_star_path[i].y + 0.5) * predicted.info.resolution;
      path_pose.pose.position.z = predicted.info.origin.position.z;
      path_pose.pose.orientation.x = 0.0;
      path_pose.pose.orientation.y = 0.0;
      path_pose.pose.orientation.z = 0.0;
      path_pose.pose.orientation.w = 1.0;
      output_path.poses.push_back(path_pose);
    }

    // publish the constructed path
    path_pub->publish(output_path);
    // if (!output_path.poses.empty()) {
    //   geometry_msgs::msg::PointStamped waypoint_msg;
    //   waypoint_msg.header = output_path.header;
    //   waypoint_msg.point = output_path.poses.back().pose.position;
    //   waypoint_pub->publish(waypoint_msg);
    // }

    DEBUG_LOG("Published A* path from (%d, %d) to (%d, %d) with %zu points", robot_x,
              robot_y, goal_x, goal_y, a_star_path.size());
  } else {
    RCLCPP_WARN(this->get_logger(), "A* search failed from (%d, %d) to (%d, %d)", robot_x,
                robot_y, goal_x, goal_y);
    nav_msgs::msg::Path output_path;
    output_path.header.stamp = this->get_clock()->now();
    output_path.header.frame_id = "map";
    path_pub->publish(output_path);
    if (curr_index < (int)exe_path.poses.size() - 1) {
      curr_index++;
    } else {
      requestPlan();
    }
  }
}

void PathSelector::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!msg || !predicted_received || !visible_received) {
    return;
  }

  // store current pose
  current_pose = msg->pose.pose;
  odom_received = true;

  double start_time = this->now().seconds();
  updateOnce();
  double end_time = this->now().seconds();
  if (DEBUG_MODE) {
    RCLCPP_INFO(this->get_logger(), "PathSelector updateOnce() took %.6f seconds",
                end_time - start_time);
  }
}

void PathSelector::predictedCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  if (!msg) {
    return;
  }
  predicted = *msg;
  predicted_received = true;
}

void PathSelector::visibleCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  if (!msg) {
    return;
  }
  visible = *msg;
  visible_cost = generateCostmap(visible);
  visible_received = true;
}

OccupancyGrid PathSelector::generateCostmap(const OccupancyGrid& original_map) {
  OccupancyGrid new_map = original_map;
  new_map.data = original_map.data;

  if (original_map.info.width <= 0 || original_map.info.height <= 0 ||
      original_map.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Invalid input map: empty or invalid dimensions");
    return new_map;
  }

  // inflate obstacles by 1 cell
  for (int y = 0; y < static_cast<int>(original_map.info.height); y++) {
    for (int x = 0; x < static_cast<int>(original_map.info.width); x++) {
      int index = y * original_map.info.width + x;
      if (original_map.data[index] == 100) { // occupied cell
        // mark 8-connected neighbors as high cost
        for (int dy = -1; dy <= 1; dy++) {
          for (int dx = -1; dx <= 1; dx++) {
            int new_y = y + dy;
            int new_x = x + dx;
            if (new_x >= 0 && new_x < static_cast<int>(original_map.info.width) &&
                new_y >= 0 && new_y < static_cast<int>(original_map.info.height)) {
              int new_index = new_y * original_map.info.width + new_x;
              if (new_map.data[new_index] != 100 && new_map.data[new_index] == 0) {
                new_map.data[new_index] = 75; // high cost but not blocked
              }
            }
          }
        }
      }
    }
  }

  return new_map;
}

bool PathSelector::aStarSearch(int start_x, int start_y, int goal_x, int goal_y,
                               std::vector<Point2D>& path) {
  if (!visible_received) {
    RCLCPP_WARN(this->get_logger(), "Map data not available for path planning");
    return false;
  }

  int width = predicted.info.width;
  int height = predicted.info.height;

  if (start_x < 0 || start_x >= width || start_y < 0 || start_y >= height || goal_x < 0 ||
      goal_x >= width || goal_y < 0 || goal_y >= height) {
    RCLCPP_INFO(this->get_logger(),
                "A* failed: Start/Goal out of bounds - Start(%d,%d) "
                "Goal(%d,%d) Map(%dx%d)",
                start_x, start_y, goal_x, goal_y, width, height);
    return false;
  }

  int start_idx = start_y * width + start_x;
  int goal_idx = goal_y * width + goal_x;
  bool start_in_obstacle =
      (visible.data[start_idx] >= 50) ||
      (visible.data[start_idx] == -1 && predicted.data[start_idx] >= 50);
  bool goal_in_obstacle =
      (visible.data[goal_idx] >= 50) ||
      (visible.data[goal_idx] == -1 && predicted.data[goal_idx] >= 50);

  if (start_in_obstacle || goal_in_obstacle) {
    RCLCPP_WARN(this->get_logger(),
                "Start (%d,%d) in obstacle: vis=%d pred=%d | Goal (%d,%d) in "
                "obstacle: vis=%d pred=%d",
                start_x, start_y, visible.data[start_idx], predicted.data[start_idx],
                goal_x, goal_y, visible.data[goal_idx], predicted.data[goal_idx]);

    // convert to world coordinates for debugging
    double start_world_x =
        predicted.info.origin.position.x + (start_x + 0.5) * predicted.info.resolution;
    double start_world_y =
        predicted.info.origin.position.y + (start_y + 0.5) * predicted.info.resolution;
    double goal_world_x =
        predicted.info.origin.position.x + (goal_x + 0.5) * predicted.info.resolution;
    double goal_world_y =
        predicted.info.origin.position.y + (goal_y + 0.5) * predicted.info.resolution;

    RCLCPP_WARN(this->get_logger(), "World coords - Start: (%.2f,%.2f) Goal: (%.2f,%.2f)",
                start_world_x, start_world_y, goal_world_x, goal_world_y);

    return false;
  }
  std::priority_queue<Node2D, std::vector<Node2D>, NodeComparator> open;
  std::unordered_set<int64_t> closed;
  std::unordered_map<int64_t, std::shared_ptr<Node2D>> nodes;
  std::unordered_map<int64_t, double> gtable;

  Node2D start(start_x, start_y, 0.0);
  start.h = std::hypot(start_x - goal_x, start_y - goal_y);
  start.f = start.g + start.h;
  int64_t start_key = start.to_key(width);
  nodes[start_key] = std::make_shared<Node2D>(start);
  gtable[start_key] = start.g;
  open.push(start);

  while (!open.empty()) {
    Node2D current = open.top();
    open.pop();
    int64_t current_key = current.to_key(width);

    if (closed.find(current_key) != closed.end()) {
      continue;
    }
    closed.insert(current_key);

    if (current.x == goal_x && current.y == goal_y) {
      std::vector<Point2D> rev;
      std::shared_ptr<Node2D> node = nodes[current_key];
      while (node) {
        rev.push_back({node->x, node->y});
        node = node->parent;
      }
      std::reverse(rev.begin(), rev.end());
      path = rev;
      return true;
    }

    for (const auto& [dir, ort] : neighbor_dirs) {
      int nx = current.x + dir.first;
      int ny = current.y + dir.second;

      if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

      size_t idx = static_cast<size_t>(ny) * width + nx;

      int8_t pred_val = predicted.data[idx];
      int8_t vis_val = visible.data[idx];

      // More permissive traversal: allow if visible is free OR predicted is not
      // too occupied
      bool is_traversable = false;
      if (vis_val == 0) {
        is_traversable = true; // Free or unknown in visible map
      } else if (vis_val == -1 && pred_val == 0) {
        is_traversable = true; // Low occupancy in predicted map
      }

      if (!is_traversable) {
        RCLCPP_DEBUG(this->get_logger(), "Cell blocked: (%d,%d) visible=%d predicted=%d",
                     nx, ny, vis_val, pred_val);
        continue;
      }

      // diagonal check
      bool is_diagonal = (dir.first != 0 && dir.second != 0);
      bool can_move = true;
      if (is_diagonal) {
        int dx = dir.first, dy = dir.second;
        int n1x = current.x + dx, n1y = current.y;
        int n2x = current.x, n2y = current.y + dy;
        if (!(n1x >= 0 && n1x < width && n1y >= 0 && n1y < height &&
              (visible.data[n1y * width + n1x] == 0 ||
               (visible.data[n1y * width + n1x] == -1 &&
                predicted.data[n1y * width + n1x] == 0))))
          can_move = false;
        if (!(n2x >= 0 && n2x < width && n2y >= 0 && n2y < height &&
              (visible.data[n2y * width + n2x] == 0 ||
               (visible.data[n2y * width + n2x] == -1 &&
                predicted.data[n2y * width + n2x] == 0))))
          can_move = false;
      }
      if (!can_move) continue;

      int64_t neighbor_key = static_cast<int64_t>(ny) * width + nx;
      if (closed.find(neighbor_key) != closed.end()) continue;

      double step = (is_diagonal ? 1.414 : 1.0);
      if (visible_cost.data[idx] > 50) {
        step += 20.0; // penalty for high-cost areas
      }

      double tentative_g = current.g + step;

      if (gtable.find(neighbor_key) == gtable.end() ||
          tentative_g < gtable[neighbor_key]) {
        gtable[neighbor_key] = tentative_g;

        Node2D neighbor(nx, ny, tentative_g);
        neighbor.h = std::hypot(nx - goal_x, ny - goal_y);
        neighbor.f = neighbor.g + neighbor.h;

        if (nodes.find(current_key) == nodes.end())
          nodes[current_key] = std::make_shared<Node2D>(current);

        neighbor.parent = nodes[current_key];
        nodes[neighbor_key] = std::make_shared<Node2D>(neighbor);

        open.push(neighbor);
      }
    }
  }
  return false;
}

nav_msgs::msg::Path PathSelector::convertPathPoseArrayToPath(
    const prob_msgs::msg::PathPoseArray& path_array) {
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";

  for (const auto& path_pose : path_array.poses) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = path.header;
    pose_stamped.pose = path_pose.pose;
    pose_stamped.pose.position.z = visible.info.origin.position.z;
    path.poses.push_back(pose_stamped);
  }

  return path;
}

void PathSelector::requestPlan() {
  if (curr_index == (int)exe_path.poses.size() - 1) {
    std_msgs::msg::Bool msg;
    msg.data = true;
    replan_requestor->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Replan requested: %d", curr_index);
    replan_requested = true;
  } else {
    std_msgs::msg::Bool msg;
    msg.data = false;
    replan_requestor->publish(msg);
    replan_requested = false;
  }
}

void PathSelector::useLocalCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg) {
    use_local = msg->data;
    if (DEBUG_MODE && use_local) {
      RCLCPP_INFO(this->get_logger(), "Received use_local = true");
    }
  }
}

void PathSelector::useGlobalCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg) {
    use_global = msg->data;
    if (DEBUG_MODE && use_global) {
      RCLCPP_INFO(this->get_logger(), "Received use_global = true");
    }
  }
}
