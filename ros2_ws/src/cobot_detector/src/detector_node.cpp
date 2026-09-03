#include "cobot_detector/detector_node.hpp"

#include <chrono>
#include <filesystem>
#include <sstream>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "cobot_interfaces/msg/key_value.hpp"
#include "cobot_interfaces/msg/obb_detection.hpp"

namespace cobot_detector
{

using namespace std::chrono_literals;
using cobot_interfaces::msg::KeyValue;
using cobot_interfaces::msg::ObbDetection;

namespace
{

std::string join(const std::vector<std::string> & v)
{
  std::string s;
  for (const auto & x : v) {
    if (!s.empty()) {s += ",";}
    s += x;
  }
  return s;
}

std::string fmt(double v, int decimals = 0)
{
  std::ostringstream ss;
  ss.setf(std::ios::fixed);
  ss.precision(decimals);
  ss << v;
  return ss.str();
}

}  // namespace

// ============================================================ construction

DetectorNode::DetectorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("detector_node", options),
  last_finished_(0, 0, RCL_STEADY_TIME)
{
  models_yaml_ = declare_parameter<std::string>(
    "models_yaml", "/ws/runtime_models/models.yaml");
  const auto preload = declare_parameter<std::vector<std::string>>(
    "preload", std::vector<std::string>{});
  heartbeat_period_s_ = declare_parameter<double>("heartbeat_period_s", 2.0);
  intra_op_threads_ = declare_parameter<int>("intra_op_threads", 0);
  declare_parameter<double>("min_confidence", -1.0);
  declare_parameter<int>("max_detections", -1);

  registry_ = ModelRegistry::load(models_yaml_);   // throws with a clear message; launch respawns
  RCLCPP_INFO(
    get_logger(), "models.yaml: %zu model(s) [%s] from %s",
    registry_.all().size(), join(registry_.ids()).c_str(), models_yaml_.c_str());

  for (const auto & id : preload) {
    if (!registry_.find(id)) {
      throw std::runtime_error("preload lists unknown model id '" + id + "'");
    }
    bool loaded_now = false;
    getOrLoad(id, loaded_now);
  }

  // ---- publishers ----
  detections_pub_ = create_publisher<ObbDetectionArray>("/vision/detections_px", 10);
  hb_pub_ = create_publisher<Heartbeat>("/health/heartbeat", rclcpp::QoS(10).reliable());

  // ---- groups ----
  hb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // ---- inputs ----
  color_sub_ = create_subscription<Image>(
    "/camera/color/image_raw", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&DetectorNode::onColor, this, std::placeholders::_1));

  // ---- action ----
  detect_server_ = rclcpp_action::create_server<Detect>(
    this, "/vision/detect",
    std::bind(&DetectorNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&DetectorNode::handleCancel, this, std::placeholders::_1),
    std::bind(&DetectorNode::handleAccepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(), action_group_);

  // ---- health ----
  hb_timer_ = create_wall_timer(
    std::chrono::duration<double>(heartbeat_period_s_),
    std::bind(&DetectorNode::heartbeatTick, this), hb_group_);

  RCLCPP_INFO(get_logger(), "detector_node ready (/vision/detect)");
}

DetectorNode::~DetectorNode()
{
  joinWorker();
}

void DetectorNode::joinWorker()
{
  std::lock_guard<std::mutex> lk(worker_mutex_);
  if (worker_.joinable()) {worker_.join();}
}

// ============================================================ inputs

void DetectorNode::onColor(const Image::ConstSharedPtr & msg)
{
  if (msg->encoding != "rgb8") {
    RCLCPP_WARN_ONCE(
      get_logger(), "Unexpected color encoding: %s (want rgb8)", msg->encoding.c_str());
    return;
  }
  if (msg->data.size() < static_cast<size_t>(msg->height) * msg->step) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "color frame data shorter than height*step; dropped");
    return;
  }
  // Wrap the message buffer (no copy), then clone so the cached frame owns
  // its pixels — the message is recycled as soon as this callback returns.
  const cv::Mat view(
    static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
    const_cast<uint8_t *>(msg->data.data()), msg->step);
  cv::Mat owned = view.clone();

  std::lock_guard<std::mutex> lk(frame_mutex_);
  frame_.rgb = std::move(owned);
  frame_.stamp = msg->header.stamp;
  frame_.received = rclcpp::Clock(RCL_STEADY_TIME).now();
  frame_.valid = true;
}

// ============================================================ models

std::shared_ptr<OnnxDetector> DetectorNode::getOrLoad(const std::string & id, bool & loaded_now)
{
  std::lock_guard<std::mutex> lk(models_mutex_);
  loaded_now = false;
  auto it = models_.find(id);
  if (it != models_.end()) {return it->second;}

  const ModelSpec * spec = registry_.find(id);
  if (!spec) {throw std::runtime_error("unknown model id '" + id + "'");}

  RCLCPP_INFO(get_logger(), "loading model '%s' (%s) from %s", id.c_str(),
    spec->obb ? "obb" : "bbox", spec->path.c_str());
  const auto t0 = std::chrono::steady_clock::now();
  auto det = std::make_shared<OnnxDetector>(*spec, intra_op_threads_);
  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();
  RCLCPP_INFO(
    get_logger(), "model '%s' loaded in %.0f ms: input %dx%d, %s head, %d class(es)%s%s, %s",
    id.c_str(), ms, det->inputWidth(), det->inputHeight(), det->layoutName(),
    det->numClasses(),
    det->classNames().empty() ? "" : " [", det->classNames().empty() ? "" :
    (join(det->classNames()) + "]").c_str(), det->providers().c_str());
  models_[id] = det;
  loaded_now = true;
  return det;
}

// ============================================================ Detect action

rclcpp_action::GoalResponse DetectorNode::handleGoal(
  const rclcpp_action::GoalUUID &, Detect::Goal::ConstSharedPtr goal)
{
  if (!registry_.find(goal->model_id)) {
    RCLCPP_WARN(
      get_logger(), "detect rejected: unknown model_id '%s' (have: %s)",
      goal->model_id.c_str(), join(registry_.ids()).c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!goal->image_source.empty()) {
    // File input (bench tests / dataset sweeps): no camera frame needed, but
    // fail fast on a bad path instead of accepting and aborting.
    if (!std::filesystem::is_regular_file(goal->image_source)) {
      RCLCPP_WARN(
        get_logger(), "detect rejected: image_source not found: %s",
        goal->image_source.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  } else {
    std::lock_guard<std::mutex> lk(frame_mutex_);
    if (!frame_.valid) {
      RCLCPP_WARN(get_logger(), "detect rejected: no camera frame received yet");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  if (in_flight_.load()) {
    RCLCPP_WARN(get_logger(), "detect rejected: a detection is already running");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DetectorNode::handleCancel(std::shared_ptr<GoalHandle>)
{
  // Inference is a few hundred ms; the worker checks is_canceling() between
  // stages and returns canceled instead of publishing.
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DetectorNode::handleAccepted(std::shared_ptr<GoalHandle> gh)
{
  bool expected = false;
  if (!in_flight_.compare_exchange_strong(expected, true)) {
    // Two goals raced past handleGoal; the loser aborts immediately.
    auto result = std::make_shared<Detect::Result>();
    result->success = false;
    result->message = "a detection is already running";
    gh->abort(result);
    return;
  }
  std::lock_guard<std::mutex> lk(worker_mutex_);
  if (worker_.joinable()) {worker_.join();}   // previous run has finished (in_flight_ was false)
  worker_ = std::thread(&DetectorNode::execute, this, gh);
}

void DetectorNode::execute(std::shared_ptr<GoalHandle> gh)
{
  const std::string id = gh->get_goal()->model_id;
  const std::string image_source = gh->get_goal()->image_source;
  auto result = std::make_shared<Detect::Result>();
  auto feedback = std::make_shared<Detect::Feedback>();
  const auto say = [&](const char * stage) {
      feedback->stage = stage;
      gh->publish_feedback(feedback);
    };
  const auto finish = [&](bool ok, const std::string & message) {
      result->success = ok;
      result->message = message;
      if (gh->is_canceling()) {
        gh->canceled(result);
      } else if (ok) {
        gh->succeed(result);
      } else {
        gh->abort(result);
      }
      in_flight_.store(false);
    };

  try {
    // Snapshot the frame the whole run will use: the cached camera frame, or
    // the file named by the goal (decoded here so a bad file aborts cleanly).
    Frame frame;
    if (!image_source.empty()) {
      const cv::Mat bgr = cv::imread(image_source, cv::IMREAD_COLOR);
      if (bgr.empty()) {
        throw std::runtime_error("could not decode image_source: " + image_source);
      }
      cv::cvtColor(bgr, frame.rgb, cv::COLOR_BGR2RGB);   // detect() wants rgb8
      frame.stamp = get_clock()->now();
      frame.received = rclcpp::Clock(RCL_STEADY_TIME).now();
      frame.valid = true;
    } else {
      std::lock_guard<std::mutex> lk(frame_mutex_);
      frame = frame_;
    }

    bool need_load = false;
    {
      std::lock_guard<std::mutex> lk(models_mutex_);
      need_load = models_.find(id) == models_.end();
    }
    if (need_load) {say("loading_model");}
    bool loaded_now = false;
    std::shared_ptr<OnnxDetector> det = getOrLoad(id, loaded_now);
    if (gh->is_canceling()) {return finish(false, "canceled before inference");}

    say("preprocess");
    say("inference");
    const float min_confidence =
      static_cast<float>(get_parameter("min_confidence").as_double());
    const int max_detections = static_cast<int>(get_parameter("max_detections").as_int());
    const OnnxDetector::Result run = det->detect(frame.rgb, min_confidence, max_detections);
    if (gh->is_canceling()) {return finish(false, "canceled after inference");}

    say("postprocess");
    ObbDetectionArray msg;
    msg.header.stamp = frame.stamp;
    msg.header.frame_id = "camera_color";
    msg.model_id = id;
    msg.image_width = static_cast<uint32_t>(frame.rgb.cols);
    msg.image_height = static_cast<uint32_t>(frame.rgb.rows);
    for (const auto & d : run.detections) {
      ObbDetection out;
      out.class_id = d.class_id;
      out.confidence = d.confidence;
      out.center_x = d.cx;
      out.center_y = d.cy;
      out.angle = d.angle;
      for (int k = 0; k < 4; ++k) {
        out.corners_px[2 * k] = d.corners[k].x;
        out.corners_px[2 * k + 1] = d.corners[k].y;
      }
      msg.detections.push_back(out);
    }

    detections_pub_->publish(msg);
    result->detections = msg;
    result->inference_ms = static_cast<float>(run.inference_ms);

    {
      std::lock_guard<std::mutex> lk(stats_mutex_);
      last_model_id_ = id;
      last_inference_ms_ = run.inference_ms;
      last_count_ = static_cast<int>(msg.detections.size());
      last_finished_ = rclcpp::Clock(RCL_STEADY_TIME).now();
      have_last_ = true;
    }
    RCLCPP_INFO(
      get_logger(), "detect '%s'%s%s: %zu detection(s), inference %.0f ms",
      id.c_str(), image_source.empty() ? "" : " on ", image_source.c_str(),
      msg.detections.size(), run.inference_ms);
    finish(true, std::to_string(msg.detections.size()) + " detection(s)");
  } catch (const std::exception & e) {   // a failed run must not kill the node
    {
      std::lock_guard<std::mutex> lk(stats_mutex_);
      ++failures_;
    }
    RCLCPP_ERROR(get_logger(), "detect '%s' failed: %s", id.c_str(), e.what());
    finish(false, e.what());
  }
}

// ============================================================ health

void DetectorNode::heartbeatTick()
{
  const auto now = rclcpp::Clock(RCL_STEADY_TIME).now();

  bool have_frame = false;
  double frame_age = -1.0;
  {
    std::lock_guard<std::mutex> lk(frame_mutex_);
    have_frame = frame_.valid;
    if (have_frame) {frame_age = (now - frame_.received).seconds();}
  }
  std::vector<std::string> loaded;
  {
    std::lock_guard<std::mutex> lk(models_mutex_);
    for (const auto & kv : models_) {loaded.push_back(kv.first);}
  }
  std::string last_id;
  double last_ms = -1.0;
  int last_count = -1;
  double last_age = -1.0;
  bool have_last = false;
  int failures = 0;
  {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    last_id = last_model_id_;
    last_ms = last_inference_ms_;
    last_count = last_count_;
    have_last = have_last_;
    failures = failures_;
    if (have_last) {last_age = (now - last_finished_).seconds();}
  }
  const bool in_flight = in_flight_.load();

  Heartbeat hb;
  hb.header.stamp = get_clock()->now();
  hb.node_name = get_name();
  hb.period_s = static_cast<float>(heartbeat_period_s_);

  if (!have_frame) {
    hb.level = Heartbeat::LEVEL_WARN;
    hb.detail = "models ready · no color frames yet";
  } else if (frame_age > 2.0) {
    hb.level = Heartbeat::LEVEL_WARN;
    hb.detail = "models ready · color frame stale " + fmt(frame_age, 1) + "s";
  } else if (in_flight) {
    hb.level = Heartbeat::LEVEL_OK;
    hb.detail = "detection in progress";
  } else if (!have_last) {
    hb.level = Heartbeat::LEVEL_OK;
    hb.detail = "idle · no detection yet";
  } else {
    hb.level = Heartbeat::LEVEL_OK;
    hb.detail = "idle · last '" + last_id + "' " + fmt(last_ms) + " ms, " +
      std::to_string(last_count) + " detections";
  }

  const auto add = [&](const std::string & k, const std::string & v) {
      KeyValue m;
      m.key = k;
      m.value = v;
      hb.metrics.push_back(m);
    };
  add("models_available", join(registry_.ids()));
  add("models_loaded", join(loaded));
  add("scan_in_flight", in_flight ? "true" : "false");
  add("last_model_id", last_id);
  add("last_scan_ms", have_last ? fmt(last_ms) : "");
  add("last_detection_count", have_last ? std::to_string(last_count) : "");
  add("last_scan_age_s", have_last ? fmt(last_age, 1) : "");
  add("scan_failures", std::to_string(failures));
  add("frame_age_s", have_frame ? fmt(frame_age, 2) : "");
  hb_pub_->publish(hb);
}

}  // namespace cobot_detector
