#include "../include/path_tracker/path_tracker.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathTracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}