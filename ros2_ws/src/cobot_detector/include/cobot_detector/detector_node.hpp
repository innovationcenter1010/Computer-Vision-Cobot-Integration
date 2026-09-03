#pragma once
// detector_node — pure perception. No geometry, no millimeters.
//
//   sub    /camera/color/image_raw   (rgb8, best-effort, latest wins) → cached
//   action /vision/detect            run one models.yaml model on the cached
//                                    frame; result = pixel detections stamped
//                                    with that frame; also mirrored on
//   pub    /vision/detections_px     (for the web overlay / debugging)
//   pub    /health/heartbeat
//
// Threading: MultiThreadedExecutor. The subscription only copies a frame
// under a mutex. Each accepted goal runs on its own std::thread so the
// executor stays free for cancel requests and the heartbeat; a second goal
// while one is executing is REJECTED (one camera, one scene, one scan).
// Models are lazy-loaded on first use (seconds) unless listed in `preload`.

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "cobot_interfaces/action/detect.hpp"
#include "cobot_interfaces/msg/heartbeat.hpp"
#include "cobot_interfaces/msg/obb_detection_array.hpp"

#include "cobot_detector/model_registry.hpp"
#include "cobot_detector/onnx_detector.hpp"

namespace cobot_detector
{

class DetectorNode : public rclcpp::Node
{
public:
  explicit DetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DetectorNode() override;

private:
  using Detect = cobot_interfaces::action::Detect;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Detect>;
  using Image = sensor_msgs::msg::Image;
  using ObbDetectionArray = cobot_interfaces::msg::ObbDetectionArray;
  using Heartbeat = cobot_interfaces::msg::Heartbeat;

  struct Frame
  {
    cv::Mat rgb;                       // owned copy
    builtin_interfaces::msg::Time stamp;
    rclcpp::Time received;             // steady, for staleness
    bool valid = false;
  };

  // ---- inputs ----
  void onColor(const Image::ConstSharedPtr & msg);

  // ---- models ----
  std::shared_ptr<OnnxDetector> getOrLoad(const std::string & id, bool & loaded_now);

  // ---- Detect action ----
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &, Detect::Goal::ConstSharedPtr goal);
  rclcpp_action::CancelResponse handleCancel(std::shared_ptr<GoalHandle>);
  void handleAccepted(std::shared_ptr<GoalHandle> gh);
  void execute(std::shared_ptr<GoalHandle> gh);
  void joinWorker();

  // ---- health ----
  void heartbeatTick();

  // parameters
  std::string models_yaml_;
  double heartbeat_period_s_ = 2.0;
  int intra_op_threads_ = 0;
  // min_confidence_ / max_detections_: < 0 (the default) means "use each
  // model's own models.yaml value". Re-read from the parameter each run so
  // `ros2 param set` takes effect on the next /vision/detect goal, no
  // restart needed.

  ModelRegistry registry_;

  std::mutex models_mutex_;
  std::map<std::string, std::shared_ptr<OnnxDetector>> models_;

  std::mutex frame_mutex_;
  Frame frame_;

  std::atomic<bool> in_flight_{false};
  std::thread worker_;
  std::mutex worker_mutex_;

  // last completed run, for the heartbeat (written under stats_mutex_)
  std::mutex stats_mutex_;
  std::string last_model_id_;
  double last_inference_ms_ = -1.0;
  int last_count_ = -1;
  rclcpp::Time last_finished_;
  bool have_last_ = false;
  int failures_ = 0;

  rclcpp::Subscription<Image>::SharedPtr color_sub_;
  rclcpp::Publisher<ObbDetectionArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<Heartbeat>::SharedPtr hb_pub_;
  rclcpp::TimerBase::SharedPtr hb_timer_;
  rclcpp_action::Server<Detect>::SharedPtr detect_server_;
  rclcpp::CallbackGroup::SharedPtr hb_group_;
  rclcpp::CallbackGroup::SharedPtr action_group_;
};

}  // namespace cobot_detector
