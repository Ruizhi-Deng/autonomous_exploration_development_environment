#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <std_srvs/srv/trigger.hpp>

bool g_stop = false;

void signal_handler(int signum) {
  (void)signum;
  g_stop = true;
}

class MapSaverNode : public rclcpp::Node {
public:
  MapSaverNode() : Node("map_saver_node") {
    client_ = this->create_client<std_srvs::srv::Trigger>("/map_save");
    RCLCPP_INFO(
        this->get_logger(),
        "Map Saver Node started. Will call /map_save on exit (Ctrl+C).");
  }

  void call_save_service() {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(),
                  "Service /map_save not available, skipping save.");
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client_->async_send_request(request);

    RCLCPP_INFO(this->get_logger(), "Calling /map_save service...");

    // Wait for the result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto result = future.get();
      if (result->success) {
        RCLCPP_INFO(this->get_logger(), "Map saved successfully: %s",
                    result->message.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save map: %s",
                     result->message.c_str());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service /map_save");
    }
  }

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

int main(int argc, char **argv) {
  // Initialize ROS 2
  // We will override the signal handler later to ensure we can perform cleanup
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MapSaverNode>();

  signal(SIGINT, signal_handler);

  rclcpp::Rate rate(10);
  while (rclcpp::ok() && !g_stop) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  // Perform the service call if we are exiting due to signal
  if (g_stop && rclcpp::ok()) {
    node->call_save_service();
  }

  rclcpp::shutdown();
  return 0;
}
