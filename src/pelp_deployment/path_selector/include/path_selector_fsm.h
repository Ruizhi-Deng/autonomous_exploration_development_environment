#ifndef PATH_SELECTOR_H
#define PATH_SELECTOR_H

#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "prob_msgs/msg/path_pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#define DEBUG_MODE false

#define DEBUG_LOG(fmt, ...)                                            \
  do                                                                   \
  {                                                                    \
    if (DEBUG_MODE)                                                    \
    {                                                                  \
      RCLCPP_INFO(this->get_logger(), fmt __VA_OPT__(, ) __VA_ARGS__); \
    }                                                                  \
  } while (0)

#define DEBUG_WARN(fmt, ...)                                           \
  do                                                                   \
  {                                                                    \
    if (DEBUG_MODE)                                                    \
    {                                                                  \
      RCLCPP_WARN(this->get_logger(), fmt __VA_OPT__(, ) __VA_ARGS__); \
    }                                                                  \
  } while (0)

using OdomSubscriberPtr =
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr;
using PathPublisherPtr = rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr;
using PathPoseSubscriberPtr =
    rclcpp::Subscription<prob_msgs::msg::PathPoseArray>::SharedPtr;
using OccupancyGridSubscriberPtr =
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr;
using ReplanPublisherPtr = rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using WaypointPublisherPtr = rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr;

struct Point2D
{
  int x, y;
  bool operator<(const Point2D &other) const
  {
    if (x != other.x)
      return x < other.x;
    return y < other.y;
  }
  bool operator==(const Point2D &other) const
  {
    return x == other.x && y == other.y;
  }
};

struct Node2D
{
  int x, y;
  double g, h, f;

  std::shared_ptr<Node2D> parent;
  Node2D(int x_, int y_, double g_ = 0.0)
      : x(x_), y(y_), g(g_), h(0.0), f(g + h), parent(nullptr) {};
  int64_t to_key(int width) const
  {
    return static_cast<int64_t>(y) * width + x;
  };
};

struct NodeComparator
{
  bool operator()(const Node2D &a, const Node2D &b) const { return a.f > b.f; };
};

class PathSelector : public rclcpp::Node
{
public:
  PathSelector();

private:
  // FSM related
  std::string selector_states[5] = {"READY", "LOCAL_INIT", "LOCAL_EXEC",
                                    "GLOBAL_INIT", "GLOBAL_EXEC"};
  enum class SelectorState
  {
    READY,
    LOCAL_INIT,
    LOCAL_EXEC,
    GLOBAL_INIT,
    GLOBAL_EXEC
  };
  SelectorState current_state = SelectorState::READY;
  void updateOnce();
  void executeExePathOnce();
  void updateCurrentIndex();

  // local planning subscriber
  PathPoseSubscriberPtr local_plan_sub;
  prob_msgs::msg::PathPoseArray local_res;
  void localPathCallback(const prob_msgs::msg::PathPoseArray::SharedPtr msg);

  // global planning subscriber
  PathPoseSubscriberPtr global_plan_sub;
  prob_msgs::msg::PathPoseArray global_res;
  void globalPathCallback(const prob_msgs::msg::PathPoseArray::SharedPtr msg);

  // subscribe to odom
  geometry_msgs::msg::Pose current_pose;
  OdomSubscriberPtr odom_subscriber;
  bool odom_received;
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // replan request
  ReplanPublisherPtr replan_requestor;
  void requestPlan();
  bool replan_requested = false;

  // use_local and use_global subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr use_local_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr use_global_sub;
  bool use_local = false;
  bool use_global = false;
  void useLocalCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void useGlobalCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // subscribe to map
  OccupancyGridSubscriberPtr predicted_subscriber, visible_subscriber;
  nav_msgs::msg::OccupancyGrid predicted, visible, visible_cost;
  void predictedCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void visibleCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  OccupancyGrid generateCostmap(const OccupancyGrid &original_map);
  bool predicted_received, visible_received;
  std::mutex map_mutex_;

  // exe path
  prob_msgs::msg::PathPoseArray exe_path;
  int curr_index;

  // publish path
  PathPublisherPtr path_pub;
  PathPublisherPtr solution_pub;

  // publish waypoint
  WaypointPublisherPtr waypoint_pub;

  // path-finding
  bool aStarSearch(int start_x, int start_y, int goal_x, int goal_y,
                   std::vector<Point2D> &path);
  // double height;

  // waypoint parameters
  double waypoint_tolerance;

  // neighbor directions for 8-connected movement
  static const std::vector<std::pair<std::pair<int, int>, double>>
      neighbor_dirs;

  // calculate yaw
  double direction_change_penalty;

  // helper function to convert PathPoseArray to nav_msgs::msg::Path
  nav_msgs::msg::Path
  convertPathPoseArrayToPath(const prob_msgs::msg::PathPoseArray &path_array);
};

#endif // PATH_SELECTOR_H