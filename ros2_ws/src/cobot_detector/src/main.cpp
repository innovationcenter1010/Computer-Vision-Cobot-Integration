#include "rclcpp/rclcpp.hpp"

#include "cobot_detector/detector_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    auto node = std::make_shared<cobot_detector::DetectorNode>();
    // 3 threads: frame subscription, heartbeat, and room for a goal/cancel
    // callback while the worker thread is busy with inference.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("detector_node"), "%s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
