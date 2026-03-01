#include "../include/path_selector_fsm.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathSelector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}