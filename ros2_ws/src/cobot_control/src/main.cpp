#include "rclcpp/rclcpp.hpp"

#include "cobot_control/cobot_control_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    auto node = std::make_shared<cobot_control::CobotControlNode>();
    // 4 threads: poll timer, heartbeat, and room for a service / action
    // callback to run while the move thread is busy.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("cobot_control"), "%s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
