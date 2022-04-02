#include "example_action/fibber.hpp"      // for Fibber
#include "example_action/rcl_handle.hpp"  // for RclHandle
#include "rclcpp/rclcpp.hpp"              // for init, spin, shutdown

#include <memory>  // for make_unique, make_shared

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fibber", rclcpp::NodeOptions{});
  auto fibber = Fibber{std::make_unique<RclHandle>(node)};
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
