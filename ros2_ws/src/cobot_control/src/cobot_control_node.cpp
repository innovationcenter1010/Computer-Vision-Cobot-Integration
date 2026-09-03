#include "cobot_control/cobot_control_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <utility>

#include "cobot_interfaces/msg/alarm.hpp"
#include "cobot_interfaces/msg/key_value.hpp"
#include "cobot_interfaces/msg/tool_file.hpp"
#include "cobot_interfaces/msg/user_frame.hpp"

namespace cobot_control
{

using namespace std::chrono_literals;
using cobot_interfaces::msg::Heartbeat;
using cobot_interfaces::msg::KeyValue;

namespace
{

// Every srv response carries (success, status_code, message).
template<class Res>
void fill(Res & res, const RobotStatus & st, const std::string & ok_text = "ok")
{
  res.success = st.ok();
  res.status_code = st.code;
  res.message = st.ok() ? (st.message.empty() ? ok_text : st.message) : st.text();
}

std::string alarmText(const std::vector<cobot_interfaces::msg::Alarm> & alarms)
{
  if (alarms.empty()) {return "alarm (no detail)";}
  std::string s;
  for (const auto & a : alarms) {
    if (!s.empty()) {s += "; ";}
    s += "alarm " + std::to_string(a.code) + "/" + std::to_string(a.subcode) + " " + a.name;
  }
  return s;
}

// The alarm this cell actually throws: a point accepted into the trajectory
// that the controller then cannot interpolate to (see the fork's
// docs/cobot_driver.md — the reachability gate + figure carry exist to turn
// this mid-motion alarm into a rejected goal).
constexpr uint32_t kInterpolationAlarm = 4684;

}  // namespace

// ============================================================ construction

CobotControlNode::CobotControlNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("cobot_control", options), last_connect_attempt_(0, 0, RCL_ROS_TIME)
{
  robot_ip_ = declare_parameter<std::string>("robot_ip", "192.168.1.31");
  setup_file_ = declare_parameter<std::string>("setup_file", "");
  poll_period_s_ = declare_parameter<double>("poll_period_s", 0.1);
  connect_retry_period_s_ = declare_parameter<double>("connect_retry_period_s", 1.0);
  const int connect_timeout_ms = declare_parameter<int>("connect_timeout_ms", 500);
  const int connect_retries = declare_parameter<int>("connect_retries", 3);
  const bool ymconnect_logging = declare_parameter<bool>("ymconnect_logging", false);
  motion_timeout_s_ = declare_parameter<double>("motion_timeout_s", 100.0);
  pause_uses_hold_ = declare_parameter<bool>("pause_uses_hold", false);
  check_reachability_ = declare_parameter<bool>("check_reachability", true);
  pendant_messages_ = declare_parameter<bool>("pendant_messages", true);
  heartbeat_period_s_ = declare_parameter<double>("heartbeat_period_s", 2.0);
  // Runtime-writable state (tooling_state.yaml, controller_profile/ dumps).
  runtime_dir_ = declare_parameter<std::string>("runtime_dir", "");
  // Controller profile: "controller" reads it over YMConnect after connect;
  // "fixture" parses profile_fixture_dir (zero-hardware path).
  profile_source_ = declare_parameter<std::string>("profile_source", "controller");
  profile_fixture_dir_ = declare_parameter<std::string>("profile_fixture_dir", "");
  profile_files_.tool_file = declare_parameter<std::string>("profile_tool_file", "TOOL.CND");
  profile_files_.uframe_file =
    declare_parameter<std::string>("profile_uframe_file", "UFRAME.CND");
  profile_dump_ = declare_parameter<bool>("profile_dump", true);
  poll_joints_ = declare_parameter<bool>("poll_joints", true);
  // --- motion wait (ported from the fork's hardware-tested cobot_driver) ---
  motion_poll_period_s_ = declare_parameter<double>("motion_poll_period_s", 0.05);
  motion_start_grace_s_ = declare_parameter<double>("motion_start_grace_s", 3.0);
  motion_feedback_hz_ = declare_parameter<double>("motion_feedback_hz", 5.0);
  joint_limit_margin_deg_ = declare_parameter<double>("joint_limit_margin_deg", 5.0);
  joint_limits_min_deg_ =
    declare_parameter<std::vector<double>>("joint_limits_min_deg", std::vector<double>{});
  joint_limits_max_deg_ =
    declare_parameter<std::vector<double>>("joint_limits_max_deg", std::vector<double>{});

  // Live tuning (dev panel → /cobot_control/set_parameters): rclcpp updates
  // its parameter store, but the checks read the cached members above — this
  // callback keeps them in sync so a change takes effect without a restart.
  // Every other parameter is refused: the rest of this constructor consumed
  // them already (connections, timers, files), so a live set would silently
  // do nothing — better to say so than pretend it applied.
  param_cb_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        const std::string & n = p.get_name();
        if (n == "joint_limit_margin_deg") {
          std::lock_guard<std::mutex> lock(joint_limits_mtx_);
          joint_limit_margin_deg_ = p.as_double();
        } else if (n == "joint_limits_min_deg" || n == "joint_limits_max_deg") {
          const std::vector<double> v = p.as_double_array();
          if (!v.empty() && v.size() != 6) {
            result.successful = false;
            result.reason = n + " needs 6 values (S L U R B T) or [] to skip the check";
            return result;
          }
          std::lock_guard<std::mutex> lock(joint_limits_mtx_);
          (n == "joint_limits_min_deg" ? joint_limits_min_deg_ : joint_limits_max_deg_) = v;
        } else {
          result.successful = false;
          result.reason = n + " is read once at startup — restart the container to apply";
          return result;
        }
      }
      return result;
    });

  if (setup_file_.empty()) {
    throw std::runtime_error(
            "setup_file parameter is required (cobot_bringup/config/cobot_setup.yaml)");
  }
  setup_ = SetupConfig::load(setup_file_);  // throws with a clear message; launch respawns
  for (const auto & d : setup_.deprecations) {
    RCLCPP_WARN(get_logger(), "%s: %s", setup_file_.c_str(), d.c_str());
  }
  RCLCPP_INFO(
    get_logger(), "setup: %zu sensors, %zu outputs, %zu motion profiles, %zu tools from %s",
    setup_.sensors.size(), setup_.outputs.size(), setup_.motion_profiles.size(),
    setup_.tools.size(), setup_file_.c_str());
  initActiveTool();

  ConnectOptions copts;
  copts.timeout_ms = static_cast<uint32_t>(std::max(connect_timeout_ms, 50));
  copts.retries = static_cast<uint32_t>(std::max(connect_retries, 1));
  copts.logging = ymconnect_logging;
  robot_ = std::make_unique<MotomanRobot>(robot_ip_, copts);

  // ---- publishers ----
  status_pub_ = create_publisher<CobotStatus>(
    "/cobot/status", rclcpp::QoS(1).reliable().transient_local());
  state_pub_ = create_publisher<CobotState>("/cobot/state", rclcpp::QoS(1).reliable());
  profile_pub_ = create_publisher<ControllerProfile>(
    "/cobot/profile", rclcpp::QoS(1).reliable().transient_local());
  hb_pub_ = create_publisher<Heartbeat>("/health/heartbeat", rclcpp::QoS(10).reliable());

  // ---- groups ----
  poll_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  hb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  io_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // ---- action ----
  move_server_ = rclcpp_action::create_server<MoveToPose>(
    this, "/cobot/move_to_pose",
    std::bind(&CobotControlNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&CobotControlNode::handleCancel, this, std::placeholders::_1),
    std::bind(&CobotControlNode::handleAccepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(), io_group_);

  // ---- services ----
  using std::placeholders::_1;
  using std::placeholders::_2;
  const auto qos = rmw_qos_profile_services_default;
  set_output_bit_srv_ = create_service<cobot_interfaces::srv::SetOutputBit>(
    "/cobot/set_output_bit", std::bind(&CobotControlNode::onSetOutputBit, this, _1, _2), qos, io_group_);
  read_io_bit_srv_ = create_service<cobot_interfaces::srv::ReadIoBit>(
    "/cobot/read_io_bit", std::bind(&CobotControlNode::onReadIoBit, this, _1, _2), qos, io_group_);
  motion_stop_srv_ = create_service<cobot_interfaces::srv::MotionStop>(
    "/cobot/motion_stop", std::bind(&CobotControlNode::onMotionStop, this, _1, _2), qos, io_group_);
  set_motion_profile_srv_ = create_service<cobot_interfaces::srv::SetMotionProfile>(
    "/cobot/set_motion_profile", std::bind(&CobotControlNode::onSetMotionProfile, this, _1, _2),
    qos, io_group_);
  read_int_var_srv_ = create_service<cobot_interfaces::srv::ReadIntVar>(
    "/cobot/read_int_var", std::bind(&CobotControlNode::onReadIntVar, this, _1, _2), qos, io_group_);
  write_int_var_srv_ = create_service<cobot_interfaces::srv::WriteIntVar>(
    "/cobot/write_int_var", std::bind(&CobotControlNode::onWriteIntVar, this, _1, _2), qos, io_group_);
  read_position_var_srv_ = create_service<cobot_interfaces::srv::ReadPositionVar>(
    "/cobot/read_position_var", std::bind(&CobotControlNode::onReadPositionVar, this, _1, _2),
    qos, io_group_);
  write_position_var_srv_ = create_service<cobot_interfaces::srv::WritePositionVar>(
    "/cobot/write_position_var", std::bind(&CobotControlNode::onWritePositionVar, this, _1, _2),
    qos, io_group_);
  get_current_pose_srv_ = create_service<cobot_interfaces::srv::GetCurrentPose>(
    "/cobot/get_current_pose", std::bind(&CobotControlNode::onGetCurrentPose, this, _1, _2),
    qos, io_group_);
  get_active_alarms_srv_ = create_service<cobot_interfaces::srv::GetActiveAlarms>(
    "/cobot/get_active_alarms", std::bind(&CobotControlNode::onGetActiveAlarms, this, _1, _2),
    qos, io_group_);
  set_active_tool_srv_ = create_service<cobot_interfaces::srv::SetActiveTool>(
    "/cobot/set_active_tool", std::bind(&CobotControlNode::onSetActiveTool, this, _1, _2),
    qos, io_group_);
  convert_pose_srv_ = create_service<cobot_interfaces::srv::ConvertPose>(
    "/cobot/convert_pose", std::bind(&CobotControlNode::onConvertPose, this, _1, _2),
    qos, io_group_);
  reload_profile_srv_ = create_service<Trigger>(
    "/cobot/reload_profile", std::bind(&CobotControlNode::onReloadProfile, this, _1, _2),
    qos, io_group_);
  pause_srv_ = create_service<Trigger>(
    "/cobot/pause", std::bind(&CobotControlNode::onPause, this, _1, _2), qos, io_group_);
  resume_srv_ = create_service<Trigger>(
    "/cobot/resume", std::bind(&CobotControlNode::onResume, this, _1, _2), qos, io_group_);
  clear_trajectory_srv_ = create_service<Trigger>(
    "/cobot/clear_trajectory", std::bind(&CobotControlNode::onClearTrajectory, this, _1, _2),
    qos, io_group_);
  clear_faults_srv_ = create_service<Trigger>(
    "/cobot/clear_faults", std::bind(&CobotControlNode::onClearFaults, this, _1, _2), qos, io_group_);
  servo_on_srv_ = create_service<Trigger>(
    "/cobot/servo_on", std::bind(&CobotControlNode::onServoOn, this, _1, _2), qos, io_group_);
  servo_off_srv_ = create_service<Trigger>(
    "/cobot/servo_off", std::bind(&CobotControlNode::onServoOff, this, _1, _2), qos, io_group_);
  reconnect_srv_ = create_service<Trigger>(
    "/cobot/reconnect", std::bind(&CobotControlNode::onReconnect, this, _1, _2), qos, io_group_);

  // ---- timers ----
  publishStatus(CobotStatus::STATE_DISCONNECTED, "connecting to " + robot_ip_, true);
  poll_timer_ = create_wall_timer(
    std::chrono::duration<double>(std::max(poll_period_s_, 0.02)),
    std::bind(&CobotControlNode::pollTick, this), poll_group_);
  hb_timer_ = create_wall_timer(
    std::chrono::duration<double>(std::max(heartbeat_period_s_, 0.2)),
    std::bind(&CobotControlNode::heartbeatTick, this), hb_group_);

  // ---- controller profile ----
  if (profile_source_ == "fixture") {
    if (profile_fixture_dir_.empty()) {
      throw std::runtime_error("profile_source=fixture needs profile_fixture_dir");
    }
    std::map<std::string, std::string> raw;
    installProfile(buildProfile(fixtureSource(profile_fixture_dir_), profile_files_, &raw), "fixture");
  } else {
    publishProfile();  // empty shell until the first connect (loaded=false)
  }

  RCLCPP_INFO(get_logger(), "cobot_control up; polling %s every %.0f ms", robot_ip_.c_str(),
    poll_period_s_ * 1000.0);
}

CobotControlNode::~CobotControlNode()
{
  if (poll_timer_) {poll_timer_->cancel();}
  if (hb_timer_) {hb_timer_->cancel();}
  if (busy_) {robot_->motionStop(true);}
  joinMoveThread();
  joinProfileThread();
  robot_.reset();  // disconnect (servos off)
}

void CobotControlNode::joinProfileThread()
{
  std::lock_guard<std::mutex> lock(profile_thread_mtx_);
  if (profile_thread_.joinable()) {profile_thread_.join();}
}

void CobotControlNode::joinMoveThread()
{
  std::lock_guard<std::mutex> lock(move_thread_mtx_);
  if (move_thread_.joinable()) {move_thread_.join();}
}

// ============================================================ status / poll

void CobotControlNode::publishStatus(uint8_t state, const std::string & detail, bool force)
{
  std::lock_guard<std::mutex> lock(status_mtx_);
  if (!force && state == last_status_state_) {return;}
  last_status_state_ = state;
  if (pendant_messages_ && robot_ && robot_->isConnected()) {
    static const char * names[] = {"DISCONNECTED", "READY", "BUSY", "ERROR"};
    const std::string tag = state < 4 ? names[state] : "?";
    robot_->pendantMessage(tag + ": " + detail);  // wrapper truncates to 30 chars
  }
  CobotStatus msg;
  msg.header.stamp = now();
  msg.state = state;
  msg.detail = detail;
  status_pub_->publish(msg);
}

void CobotControlNode::publishState(const CobotState & msg)
{
  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    last_state_ = msg;
    ++poll_seq_;
  }
  poll_cv_.notify_all();
  state_pub_->publish(msg);
}

bool CobotControlNode::tryConnect()
{
  const rclcpp::Time t = now();
  if (connect_attempts_ > 0 &&
    (t - last_connect_attempt_).seconds() < connect_retry_period_s_)
  {
    return false;
  }
  last_connect_attempt_ = t;
  ++connect_attempts_;
  const RobotStatus st = robot_->connect();
  if (!st.ok()) {
    if (connect_attempts_ == 1 || connect_attempts_ % 10 == 0) {
      RCLCPP_WARN(
        get_logger(), "connect attempt %d to %s failed: %s", connect_attempts_,
        robot_ip_.c_str(), st.text().c_str());
    }
    publishStatus(CobotStatus::STATE_DISCONNECTED, st.text());
    return false;
  }
  RCLCPP_INFO(get_logger(), "connected to %s: %s", robot_ip_.c_str(), st.message.c_str());
  connect_attempts_ = 0;
  publishStatus(CobotStatus::STATE_IDLE, st.message, true);
  startProfileRead();  // read-once controller profile (idempotent per connect)
  return true;
}

void CobotControlNode::onLinkLost(const std::string & why)
{
  RCLCPP_ERROR(get_logger(), "robot link lost: %s", why.c_str());
  robot_->disconnect(false);
  held_ = false;
  publishStatus(CobotStatus::STATE_DISCONNECTED, "robot link lost: " + why, true);
}

void CobotControlNode::pollTick()
{
  const auto t0 = std::chrono::steady_clock::now();

  if (!robot_->isConnected()) {
    if (!tryConnect()) {
      CobotState msg;
      msg.header.stamp = now();
      msg.connected = false;
      msg.paused = paused_;
      msg.busy = busy_;
      {
        const ActiveTool t = activeTool();  // selection is visible even offline
        msg.active_tool_number = t.number;
        msg.active_tool_name = t.name;
        msg.pose_tool_number = t.number;
      }
      {
        std::lock_guard<std::mutex> lock(profile_mtx_);
        msg.profile_loaded = profile_.loaded;
      }
      publishState(msg);
      return;
    }
  }

  CobotState msg;
  msg.header.stamp = now();
  msg.connected = true;

  ControllerState cs;
  RobotStatus st = robot_->readState(cs);
  if (!st.ok()) {
    onLinkLost(st.text());
    msg.connected = false;
    publishState(msg);
    return;
  }
  msg.servo_on = cs.servo_on;
  msg.running = cs.running;
  msg.in_hold = cs.in_hold;
  msg.alarming = cs.alarming;
  msg.erroring = cs.erroring;
  msg.control_mode = static_cast<uint8_t>(cs.control_mode);

  // One ReadByte per distinct (kind, group) covers every named bit in it.
  std::map<std::pair<int, uint16_t>, uint8_t> bytes;
  auto byteFor = [&](const IoAddress & a, bool & ok) -> uint8_t {
      const auto key = std::make_pair(static_cast<int>(a.kind), a.group);
      auto it = bytes.find(key);
      if (it != bytes.end()) {ok = true; return it->second;}
      uint8_t v = 0;
      const RobotStatus r = robot_->readIoByte(a.kind, a.group, v);
      ok = r.ok();
      if (!ok) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "read %s group %u failed: %s",
          ioKindName(a.kind), a.group, r.text().c_str());
      }
      bytes[key] = v;
      return v;
    };
  {
    std::lock_guard<std::mutex> lock(setup_mtx_);
    for (const auto & a : setup_.sensors) {
      bool ok = false;
      const uint8_t b = byteFor(a, ok);
      msg.sensor_names.push_back(a.name);
      msg.sensor_values.push_back(ok && ((b >> a.bit) & 1u));
    }
    for (const auto & a : setup_.outputs) {
      bool ok = false;
      const uint8_t b = byteFor(a, ok);
      msg.output_names.push_back(a.name);
      msg.output_values.push_back(ok && ((b >> a.bit) & 1u));
    }
  }

  {
    const ActiveTool t = activeTool();
    msg.active_tool_number = t.number;
    msg.active_tool_name = t.name;
    msg.pose_tool_number = t.number;
    Pose p;
    std::vector<double> joints;
    const RobotStatus pst = poll_joints_ ?
      robot_->readCurrentPoseAndJoints(t.number, p, joints) :
      robot_->readCurrentPose(t.number, p);
    if (pst.ok()) {
      msg.x = p.x; msg.y = p.y; msg.z = p.z;
      msg.rx = p.rx; msg.ry = p.ry; msg.rz = p.rz;
      msg.joints_deg = joints;
    }
  }
  {
    std::lock_guard<std::mutex> lock(profile_mtx_);
    msg.profile_loaded = profile_.loaded;
  }

  if (cs.alarming) {
    std::vector<Alarm> alarms;
    if (robot_->activeAlarms(alarms).ok()) {
      for (const auto & a : alarms) {
        cobot_interfaces::msg::Alarm m;
        m.code = a.code; m.subcode = a.subcode; m.name = a.name; m.time = a.time;
        msg.alarms.push_back(m);
      }
    }
  }

  // Motion progress is read FRESH by the move loop itself (fork-style) —
  // the poll no longer caches a sample the loop could act on stale.
  if (!busy_) {
    // Idle: surface controller-side problems on the latched status.
    if (cs.alarming) {
      publishStatus(CobotStatus::STATE_ERROR, alarmText(msg.alarms));
    } else if (!cs.servo_on) {
      publishStatus(CobotStatus::STATE_ERROR, "servos off");
    } else if (cs.control_mode == CobotState::CONTROL_MODE_TEACH) {
      publishStatus(CobotStatus::STATE_ERROR, "controller in TEACH mode");
    } else {
      publishStatus(CobotStatus::STATE_IDLE, "ready");
    }
  }

  msg.paused = paused_;
  msg.busy = busy_;
  msg.poll_ms = static_cast<float>(
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
  publishState(msg);
}

void CobotControlNode::heartbeatTick()
{
  CobotState s;
  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    s = last_state_;
  }
  Heartbeat hb;
  hb.header.stamp = now();
  hb.node_name = get_name();
  hb.period_s = static_cast<float>(std::max(heartbeat_period_s_, 0.2));
  if (!s.connected) {
    hb.level = Heartbeat::LEVEL_ERROR;
    hb.detail = "not connected (" + robot_ip_ + ")";
  } else if (s.alarming) {
    hb.level = Heartbeat::LEVEL_ERROR;
    hb.detail = alarmText(s.alarms);
  } else if (!s.servo_on) {
    hb.level = Heartbeat::LEVEL_WARN;
    hb.detail = "servos off";
  } else {
    hb.level = Heartbeat::LEVEL_OK;
    hb.detail = busy_ ? (paused_ ? "connected · paused" : "connected · busy") :
      "connected · idle";
  }
  auto kv = [&](const std::string & k, const std::string & v) {
      KeyValue m; m.key = k; m.value = v; hb.metrics.push_back(m);
    };
  kv("connected", s.connected ? "true" : "false");
  kv("busy", busy_ ? "true" : "false");
  kv("paused", paused_ ? "true" : "false");
  kv("servo_on", s.servo_on ? "true" : "false");
  kv("alarming", s.alarming ? "true" : "false");
  kv("poll_ms", std::to_string(static_cast<int>(s.poll_ms)));
  kv("robot_ip", robot_ip_);
  {
    const ActiveTool t = activeTool();
    kv("active_tool", t.name);
    kv("active_tool_number", std::to_string(t.number));
    kv("active_tool_verified", t.verified ? "true" : "false");
  }
  kv("profile_loaded", s.profile_loaded ? "true" : "false");
  hb_pub_->publish(hb);
}

// ============================================================ MoveToPose

MotionProfile CobotControlNode::resolveProfile(
  const MoveToPose::Goal & goal, std::string & profile_name, int32_t & tool_number,
  double & mount_yaw_offset_deg)
{
  // The tool file is NOT part of the profile: goal.tool_number >= 0 (a pose
  // stored with its own coordinate system) wins, otherwise the active tool.
  tool_number = goal.tool_number >= 0 ? goal.tool_number : active_tool_number_.load();
  std::lock_guard<std::mutex> lock(setup_mtx_);
  mount_yaw_offset_deg = setup_.mount_yaw_offset_deg;
  profile_name = goal.motion_profile.empty() ? "default" : goal.motion_profile;
  auto it = setup_.motion_profiles.find(profile_name);
  if (it == setup_.motion_profiles.end()) {
    RCLCPP_WARN(
      get_logger(), "unknown motion_profile '%s' — using default", profile_name.c_str());
    profile_name = "default";
    it = setup_.motion_profiles.find("default");
  }
  MotionProfile p = it->second;
  if (goal.interpolation == MoveToPose::Goal::INTERP_JOINT) {p.interpolation = Interp::Joint;}
  if (goal.interpolation == MoveToPose::Goal::INTERP_LINEAR) {p.interpolation = Interp::Linear;}
  if (goal.speed > 0.0) {p.speed = goal.speed;}
  if (goal.accuracy_mm >= -1) {p.accuracy_mm = goal.accuracy_mm;}
  return p;
}

rclcpp_action::GoalResponse CobotControlNode::handleGoal(
  const rclcpp_action::GoalUUID &, MoveToPose::Goal::ConstSharedPtr goal)
{
  if (!robot_->isConnected()) {
    RCLCPP_WARN(get_logger(), "goal rejected: robot not connected");
    return rclcpp_action::GoalResponse::REJECT;
  }
  CobotState s;
  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    s = last_state_;
  }
  if (s.alarming) {
    RCLCPP_WARN(get_logger(), "goal rejected: %s", alarmText(s.alarms).c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!s.servo_on) {
    RCLCPP_WARN(get_logger(), "goal rejected: servos off");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (s.control_mode == CobotState::CONTROL_MODE_TEACH) {
    RCLCPP_WARN(get_logger(), "goal rejected: controller in TEACH mode");
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!busy_.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN(get_logger(), "goal rejected: a motion goal is already executing");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(
    get_logger(), "move_to_pose accepted (%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)", goal->x, goal->y,
    goal->z, goal->rx, goal->ry, goal->rz);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CobotControlNode::handleCancel(std::shared_ptr<GoalHandle>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CobotControlNode::handleAccepted(std::shared_ptr<GoalHandle> gh)
{
  std::lock_guard<std::mutex> lock(move_thread_mtx_);
  if (move_thread_.joinable()) {move_thread_.join();}  // previous move finished (busy_ was false)
  move_thread_ = std::thread(&CobotControlNode::executeMove, this, gh);
}

void CobotControlNode::holdMotion()
{
  if (held_.exchange(true)) {return;}
  const RobotStatus st = pause_uses_hold_ ? robot_->setHold(true) : robot_->motionStop(false);
  if (!st.ok()) {
    RCLCPP_ERROR(get_logger(), "pause: %s", st.text().c_str());
  } else {
    RCLCPP_INFO(get_logger(), "motion held (pause)");
  }
}

void CobotControlNode::releaseMotion()
{
  if (!held_.exchange(false)) {return;}
  const RobotStatus st = pause_uses_hold_ ? robot_->setHold(false) : robot_->motionStart();
  if (!st.ok()) {
    RCLCPP_ERROR(get_logger(), "resume: %s", st.text().c_str());
  } else {
    RCLCPP_INFO(get_logger(), "motion resumed");
  }
}

bool CobotControlNode::jointLimitsOk(
  const std::vector<double> & joints_deg, std::string & detail) const
{
  // Ported from the fork's pose_validator (limits are PARAMETERS — the old
  // hardcoded table listed the T axis as -2..+2 deg, which no wrist has).
  // Empty limits = check skipped; the controller's IK answer stands alone.
  static const char * kAxisNames[] = {"S", "L", "U", "R", "B", "T"};
  // Copy under the lock: the set_parameters callback may swap these while a
  // motion is being validated on another executor thread.
  double margin;
  std::vector<double> lim_min, lim_max;
  {
    std::lock_guard<std::mutex> lock(joint_limits_mtx_);
    margin = joint_limit_margin_deg_;
    lim_min = joint_limits_min_deg_;
    lim_max = joint_limits_max_deg_;
  }
  const size_t count = std::min(joints_deg.size(), std::min(lim_min.size(), lim_max.size()));
  for (size_t i = 0; i < count; ++i) {
    const double to_min = joints_deg[i] - lim_min[i] - margin;
    const double to_max = lim_max[i] - joints_deg[i] - margin;
    if (std::min(to_min, to_max) < 0.0) {
      char buf[160];
      std::snprintf(
        buf, sizeof(buf), "joint %s = %.1f deg is inside the %.1f deg margin of [%.1f, %.1f]",
        i < 6 ? kAxisNames[i] : "?", joints_deg[i], margin, lim_min[i], lim_max[i]);
      detail = buf;
      return false;
    }
  }
  return true;
}

void CobotControlNode::executeMove(std::shared_ptr<GoalHandle> gh)
{
  const auto goal = gh->get_goal();
  auto result = std::make_shared<MoveToPose::Result>();
  auto feedback = std::make_shared<MoveToPose::Feedback>();
  std::string profile_name;
  int32_t tool = 0;
  double mount_yaw_offset_deg = 0.0;
  const MotionProfile prof = resolveProfile(*goal, profile_name, tool, mount_yaw_offset_deg);
  // Every commanded pose is corrected here, regardless of caller (choreography
  // moves, teach points, calibration grid) — see SetupConfig::mount_yaw_offset_deg.
  // Only rz (T) wraps: it's the only axis the offset is added to, and the
  // only one with a 360 deg periodic Euler/joint convention (rx/ry are fixed
  // by the pick orientation; S/L/U/R/B are untouched here).
  const Pose pose{
    goal->x, goal->y, goal->z, goal->rx, goal->ry,
    normalizeDeg180(goal->rz + mount_yaw_offset_deg)};

  stop_requested_ = false;
  publishStatus(CobotStatus::STATE_BUSY, "move_to_pose " + pose.str(), true);

  auto abort = [&](int32_t code, const std::string & m) {
      result->success = false;
      result->status_code = code;
      result->message = m;
      RCLCPP_WARN(get_logger(), "move %s aborted: %s", pose.str().c_str(), m.c_str());
      gh->abort(result);
    };
  auto canceled = [&](const std::string & m) {
      result->success = false;
      result->status_code = 0;
      result->message = m;
      RCLCPP_INFO(get_logger(), "move %s canceled: %s", pose.str().c_str(), m.c_str());
      gh->canceled(result);
    };
  auto finish = [&]() {
      held_ = false;
      stop_requested_ = false;
      busy_ = false;
      if (robot_->isConnected()) {
        publishStatus(CobotStatus::STATE_IDLE, "ready", true);
      }
    };

  // Pause latch set while idle: hold before we even queue the point.
  while (paused_ && !gh->is_canceling() && rclcpp::ok()) {
    feedback->paused = true;
    gh->publish_feedback(feedback);
    std::this_thread::sleep_for(100ms);
  }
  if (gh->is_canceling()) {canceled("canceled before start"); finish(); return;}

  if (check_reachability_) {
    bool reachable = false;
    std::string why;
    std::vector<double> solution_deg;
    const RobotStatus st = robot_->isReachable(pose, tool, reachable, why, solution_deg);
    if (!st.ok()) {abort(st.code, "reachability check: " + st.text()); finish(); return;}
    if (!reachable) {abort(2, "unreachable pose: " + why); finish(); return;}
    std::string limit_detail;
    if (!jointLimitsOk(solution_deg, limit_detail)) {
      abort(2, "reachable but too close to a joint limit: " + limit_detail);
      finish();
      return;
    }
  }

  RobotStatus st = robot_->addPoint(pose, prof, tool);
  if (!st.ok()) {abort(st.code, "add point: " + st.text()); finish(); return;}
  st = robot_->motionStart();
  if (!st.ok()) {
    robot_->clearTrajectory();
    abort(st.code, "motion start: " + st.text());
    finish();
    return;
  }
  RCLCPP_INFO(
    get_logger(), "moving to %s [%s %s speed=%.1f tool=%d]", pose.str().c_str(),
    profile_name.c_str(), prof.interpolation == Interp::Linear ? "linear" : "joint", prof.speed,
    tool);

  // ---- the wait: ported from the fork's hardware-tested run_motion ----
  // (cobot-vision-development, cobot_driver_node.cpp). The loop reads the
  // controller DIRECTLY each iteration — state and steps must be fresh,
  // never the poll's cached sample (a stale sample once meant a false
  // "reached pose" right after MotionStart). Completion = buffer empty AND
  // the controller reporting stopped, after motion was actually observed to
  // start; the old "progress >= 99% && steps <= 1" reported arrival while
  // still decelerating and accepted one whole unexecuted point.
  const auto poll = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(std::max(motion_poll_period_s_, 0.005)));
  const auto feedback_period =
    std::chrono::duration<double>(1.0 / std::max(motion_feedback_hz_, 0.5));
  const auto grace = std::chrono::duration<double>(std::max(motion_start_grace_s_, 0.1));
  constexpr int32_t queued_count = 1;

  auto begin = std::chrono::steady_clock::now();
  auto segment_start = begin;             // moving time only — pause resets it
  auto last_feedback = begin - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    feedback_period);
  bool observed_start = false;
  int32_t last_steps = queued_count;
  double last_percent = 0.0;

  while (rclcpp::ok()) {
    if (gh->is_canceling()) {
      robot_->motionStop(true);
      robot_->clearTrajectory();
      canceled("canceled: motion stopped");
      break;
    }
    if (stop_requested_) {
      abort(3, "stopped by /cobot/motion_stop");
      break;
    }

    // --- controller state; alarms come first ------------------------------
    // An alarm mid-trajectory (4684 interpolation being the one this cell
    // hits) stops the arm at the controller, but the buffer still holds
    // points and the run state can linger. Catch it here and report the
    // code instead of waiting out the timeout.
    ControllerState cs;
    const RobotStatus state_st = robot_->readState(cs);
    if (!state_st.ok()) {
      abort(1, "lost contact with the controller during motion: " + state_st.text());
      break;
    }
    if (cs.alarming || cs.erroring) {
      robot_->motionStop(true);
      robot_->clearTrajectory();
      std::vector<Alarm> alarms;
      robot_->activeAlarms(alarms);
      std::string m = "controller alarm";
      bool interpolation = false;
      for (const auto & a : alarms) {
        m += " " + std::to_string(a.code) + "/" + std::to_string(a.subcode) +
          (a.name.empty() ? "" : " (" + a.name + ")");
        interpolation = interpolation || a.code == kInterpolationAlarm;
      }
      if (interpolation) {
        m += " — the point was accepted but could not be interpolated to; the "
          "reachability gate passed a pose the motion planner rejects "
          "(IK branch changed mid-path?)";
      }
      abort(4, m);
      break;
    }
    if (!cs.servo_on && observed_start) {
      robot_->motionStop(true);
      abort(7, "servos dropped out mid-motion");
      break;
    }

    // --- pause ------------------------------------------------------------
    if (paused_) {
      holdMotion();
      feedback->paused = true;
      feedback->progress_percent = static_cast<float>(last_percent);
      feedback->steps_remaining = last_steps;
      gh->publish_feedback(feedback);
      std::this_thread::sleep_for(poll);
      segment_start = std::chrono::steady_clock::now();
      if (!observed_start) {begin = segment_start;}  // grace excludes the pause
      continue;
    }
    if (held_) {
      releaseMotion();
      continue;
    }

    // --- progress: fresh reads, never a cache -----------------------------
    MotionProgress prog;
    const RobotStatus prog_st = robot_->motionProgress(prog);
    if (!prog_st.ok()) {
      robot_->motionStop(true);
      abort(prog_st.code, "progress read: " + prog_st.text());
      break;
    }
    last_steps = prog.steps_remaining;
    last_percent = prog.percent;

    if (!observed_start &&
      (cs.running || prog.steps_remaining < queued_count || prog.percent > 0.0))
    {
      observed_start = true;
    }

    // --- completion: buffer empty AND controller stopped ------------------
    if (observed_start && prog.steps_remaining == 0 && !cs.running) {
      result->success = true;
      result->status_code = 0;
      result->message = "reached pose";
      gh->succeed(result);
      RCLCPP_INFO(get_logger(), "reached %s", pose.str().c_str());
      break;
    }

    const auto now_tp = std::chrono::steady_clock::now();
    if (!observed_start && now_tp - begin > grace) {
      if (prog.steps_remaining == 0) {
        // Nothing to do — the controller consumed the trajectory immediately.
        result->success = true;
        result->status_code = 0;
        result->message = "already at target";
        gh->succeed(result);
        break;
      }
      robot_->motionStop(true);
      robot_->clearTrajectory();
      abort(
        8, "motion did not start within " + std::to_string(motion_start_grace_s_) +
        " s — check servos, HOLD, and that the pendant is in REMOTE or PLAY");
      break;
    }

    const double moving_s = std::chrono::duration<double>(now_tp - segment_start).count();
    if (moving_s > motion_timeout_s_) {
      robot_->motionStop(true);
      robot_->clearTrajectory();
      abort(
        5, "motion timeout after " + std::to_string(static_cast<int>(moving_s)) + " s with " +
        std::to_string(last_steps) + " point(s) still queued");
      break;
    }

    if (now_tp - last_feedback > feedback_period) {
      last_feedback = now_tp;
      feedback->paused = false;
      feedback->progress_percent = static_cast<float>(prog.percent);
      feedback->steps_remaining = prog.steps_remaining;
      gh->publish_feedback(feedback);
    }
    std::this_thread::sleep_for(poll);
  }
  if (!rclcpp::ok() && gh->is_active()) {
    robot_->motionStop(true);
    abort(6, "node shutting down");
  }
  finish();
}

// ============================================================ services

bool CobotControlNode::resolveIo(
  const std::string & name, uint8_t io_type, uint16_t group, uint8_t bit, bool want_output,
  IoAddress & out, std::string & err) const
{
  std::lock_guard<std::mutex> lock(setup_mtx_);
  if (!name.empty()) {
    const IoAddress * a = want_output ? setup_.findOutput(name) : setup_.findSensor(name);
    if (!a && !want_output) {a = setup_.findOutput(name);}
    if (!a) {
      err = "unknown io name '" + name + "' (see cobot_setup.yaml io." +
        (want_output ? "outputs" : "sensors") + ")";
      return false;
    }
    out = *a;
    return true;
  }
  if (bit > 7) {err = "bit must be 0-7"; return false;}
  out.name = "raw";
  out.kind = io_type == 0 ? IoKind::GeneralInput : IoKind::GeneralOutput;
  out.group = group;
  out.bit = bit;
  return true;
}

void CobotControlNode::onSetOutputBit(
  cobot_interfaces::srv::SetOutputBit::Request::ConstSharedPtr req,
  cobot_interfaces::srv::SetOutputBit::Response::SharedPtr res)
{
  IoAddress a;
  std::string err;
  if (!resolveIo(req->name, req->io_type, req->group, req->bit, true, a, err)) {
    fill(*res, RobotStatus{2, err});
    return;
  }
  if (a.kind != IoKind::GeneralOutput) {
    fill(*res, RobotStatus{2, "'" + a.name + "' is an input, cannot write"});
    return;
  }
  const RobotStatus st = robot_->writeOutputBit(a.group, a.bit, req->value);
  fill(*res, st, a.name + " := " + (req->value ? "1" : "0"));
  if (st.ok()) {
    RCLCPP_INFO(get_logger(), "output %s (G%u:B%u) := %d", a.name.c_str(), a.group, a.bit, req->value);
  }
}

void CobotControlNode::onReadIoBit(
  cobot_interfaces::srv::ReadIoBit::Request::ConstSharedPtr req,
  cobot_interfaces::srv::ReadIoBit::Response::SharedPtr res)
{
  IoAddress a;
  std::string err;
  if (!resolveIo(req->name, req->io_type, req->group, req->bit, false, a, err)) {
    fill(*res, RobotStatus{2, err});
    return;
  }
  bool v = false;
  const RobotStatus st = robot_->readIoBit(a.kind, a.group, a.bit, v);
  res->value = v;
  fill(*res, st, a.name);
}

void CobotControlNode::onMotionStop(
  cobot_interfaces::srv::MotionStop::Request::ConstSharedPtr req,
  cobot_interfaces::srv::MotionStop::Response::SharedPtr res)
{
  const RobotStatus st = robot_->motionStop(req->clear_trajectory);
  if (busy_) {stop_requested_ = true;}
  held_ = false;
  fill(*res, st, req->clear_trajectory ? "motion stopped, trajectory cleared" : "motion stopped");
  RCLCPP_WARN(get_logger(), "motion_stop(clear=%d): %s", req->clear_trajectory, res->message.c_str());
}

void CobotControlNode::onPause(Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res)
{
  paused_ = true;
  if (busy_) {holdMotion();}  // immediate; the move loop keeps it held
  res->success = true;
  res->message = busy_ ? "paused (motion held)" : "paused (next move will wait)";
  RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
}

void CobotControlNode::onResume(Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res)
{
  paused_ = false;
  if (busy_) {releaseMotion();}
  res->success = true;
  res->message = "resumed";
  RCLCPP_INFO(get_logger(), "resumed");
}

void CobotControlNode::onClearTrajectory(
  Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res)
{
  if (busy_) {
    res->success = false;
    res->message = "a motion goal is executing — use /cobot/motion_stop";
    return;
  }
  const RobotStatus st = robot_->clearTrajectory();
  res->success = st.ok();
  res->message = st.ok() ? "trajectory cleared" : st.text();
}

void CobotControlNode::onClearFaults(
  Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res)
{
  const RobotStatus st = robot_->clearFaults();
  res->success = st.ok();
  res->message = st.ok() ? "faults cleared" : st.text();
}

void CobotControlNode::onServoOn(Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res)
{
  const RobotStatus st = robot_->setServos(true);
  res->success = st.ok();
  res->message = st.ok() ? "servos on" : st.text();
}

void CobotControlNode::onServoOff(Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res)
{
  if (busy_) {
    res->success = false;
    res->message = "a motion goal is executing — stop it first";
    return;
  }
  const RobotStatus st = robot_->setServos(false);
  res->success = st.ok();
  res->message = st.ok() ? "servos off" : st.text();
}

void CobotControlNode::onReconnect(
  Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res)
{
  if (busy_) {
    res->success = false;
    res->message = "a motion goal is executing — stop it first";
    return;
  }
  robot_->disconnect(true);
  publishStatus(CobotStatus::STATE_DISCONNECTED, "reconnecting", true);
  connect_attempts_ = 0;
  const bool ok = tryConnect();
  res->success = ok;
  res->message = ok ? "reconnected" : "reconnect failed — poll keeps retrying";
}

void CobotControlNode::onSetMotionProfile(
  cobot_interfaces::srv::SetMotionProfile::Request::ConstSharedPtr req,
  cobot_interfaces::srv::SetMotionProfile::Response::SharedPtr res)
{
  using Req = cobot_interfaces::srv::SetMotionProfile::Request;
  std::lock_guard<std::mutex> lock(setup_mtx_);
  const std::string name = req->name.empty() ? "default" : req->name;
  MotionProfile p = setup_.motion_profiles.count(name) ? setup_.motion_profiles[name] :
    setup_.motion_profiles["default"];
  if (req->interpolation == Req::INTERP_JOINT) {p.interpolation = Interp::Joint;}
  if (req->interpolation == Req::INTERP_LINEAR) {p.interpolation = Interp::Linear;}
  if (req->speed > 0.0) {p.speed = req->speed;}
  if (req->accel_ratio > 0.0) {p.accel_ratio = req->accel_ratio;}
  if (req->decel_ratio > 0.0) {p.decel_ratio = req->decel_ratio;}
  if (req->accuracy_mm >= -1) {p.accuracy_mm = req->accuracy_mm;}
  if (p.accel_ratio < 20.0 || p.accel_ratio > 100.0 || p.decel_ratio < 20.0 ||
    p.decel_ratio > 100.0)
  {
    fill(*res, RobotStatus{2, "accel/decel ratio must be 20-100"});
    return;
  }
  setup_.motion_profiles[name] = p;
  fill(*res, RobotStatus::okay("profile '" + name + "' updated"));
  RCLCPP_INFO(
    get_logger(), "motion profile '%s': %s speed=%.1f accel=%.0f decel=%.0f acc=%d",
    name.c_str(), p.interpolation == Interp::Linear ? "linear" : "joint", p.speed, p.accel_ratio,
    p.decel_ratio, p.accuracy_mm);
}

void CobotControlNode::onReadIntVar(
  cobot_interfaces::srv::ReadIntVar::Request::ConstSharedPtr req,
  cobot_interfaces::srv::ReadIntVar::Response::SharedPtr res)
{
  int32_t v = 0;
  const RobotStatus st = robot_->readInt(req->index, v);
  res->value = v;
  fill(*res, st);
}

void CobotControlNode::onWriteIntVar(
  cobot_interfaces::srv::WriteIntVar::Request::ConstSharedPtr req,
  cobot_interfaces::srv::WriteIntVar::Response::SharedPtr res)
{
  fill(*res, robot_->writeInt(req->index, req->value));
}

void CobotControlNode::onReadPositionVar(
  cobot_interfaces::srv::ReadPositionVar::Request::ConstSharedPtr req,
  cobot_interfaces::srv::ReadPositionVar::Response::SharedPtr res)
{
  Pose p;
  int32_t tool = 0;
  const RobotStatus st = robot_->readPositionVar(req->index, p, tool);
  res->x = p.x; res->y = p.y; res->z = p.z; res->rx = p.rx; res->ry = p.ry; res->rz = p.rz;
  res->tool_number = tool;
  fill(*res, st);
}

void CobotControlNode::onWritePositionVar(
  cobot_interfaces::srv::WritePositionVar::Request::ConstSharedPtr req,
  cobot_interfaces::srv::WritePositionVar::Response::SharedPtr res)
{
  const Pose p{req->x, req->y, req->z, req->rx, req->ry, req->rz};
  const int32_t tool = req->tool_number >= 0 ? req->tool_number : active_tool_number_.load();
  fill(*res, robot_->writePositionVar(req->index, p, tool));
}

void CobotControlNode::onGetCurrentPose(
  cobot_interfaces::srv::GetCurrentPose::Request::ConstSharedPtr req,
  cobot_interfaces::srv::GetCurrentPose::Response::SharedPtr res)
{
  const int32_t tool = req->tool_number >= 0 ? req->tool_number : active_tool_number_.load();
  Pose p;
  std::vector<double> joints;
  const RobotStatus st = robot_->readCurrentPoseAndJoints(tool, p, joints);
  res->x = p.x; res->y = p.y; res->z = p.z; res->rx = p.rx; res->ry = p.ry; res->rz = p.rz;
  res->tool_number = tool;
  res->joints_deg = joints;
  fill(*res, st, p.str() + " tool " + std::to_string(tool));
}

void CobotControlNode::onGetActiveAlarms(
  cobot_interfaces::srv::GetActiveAlarms::Request::ConstSharedPtr,
  cobot_interfaces::srv::GetActiveAlarms::Response::SharedPtr res)
{
  std::vector<Alarm> alarms;
  const RobotStatus st = robot_->activeAlarms(alarms);
  for (const auto & a : alarms) {
    cobot_interfaces::msg::Alarm m;
    m.code = a.code; m.subcode = a.subcode; m.name = a.name; m.time = a.time;
    res->alarms.push_back(m);
  }
  fill(*res, st, std::to_string(alarms.size()) + " active alarm(s)");
}

// ============================================================ active tool

ActiveTool CobotControlNode::activeTool() const
{
  std::lock_guard<std::mutex> lock(tool_mtx_);
  return active_tool_;
}

void CobotControlNode::setActiveTool(const ActiveTool & tool)
{
  std::lock_guard<std::mutex> lock(tool_mtx_);
  active_tool_ = tool;
  active_tool_number_ = tool.number;
}

void CobotControlNode::initActiveTool()
{
  ActiveTool t;
  t.name = setup_.active_tool_default;
  t.number = 0;
  if (t.name != "none") {
    if (const ToolBinding * b = setup_.findTool(t.name)) {t.number = b->tool_number;}
  }
  t.source = "setup_default";

  if (runtime_dir_.empty()) {
    RCLCPP_WARN(
      get_logger(), "runtime_dir is unset: the active tool selection will not persist "
      "across restarts (booting with '%s' from cobot_setup.yaml)", t.name.c_str());
  } else {
    tooling_state_path_ = (std::filesystem::path(runtime_dir_) / "tooling_state.yaml").string();
    ActiveTool saved;
    std::string err;
    if (ToolingState::load(tooling_state_path_, saved, err)) {
      if (saved.name == "none") {
        saved.number = 0;
      } else if (const ToolBinding * b = setup_.findTool(saved.name)) {
        if (b->tool_number != saved.number) {
          RCLCPP_WARN(
            get_logger(), "tooling_state.yaml says '%s' is tool file %d but cobot_setup.yaml "
            "says %d — using %d", saved.name.c_str(), saved.number, b->tool_number,
            b->tool_number);
          saved.number = b->tool_number;
        }
      } else {
        RCLCPP_WARN(
          get_logger(), "tooling_state.yaml names tool '%s' (file %d) which is not in "
          "cobot_setup.yaml tools — kept until verified against the controller",
          saved.name.c_str(), saved.number);
      }
      t = saved;
      t.source = "state_file";
      RCLCPP_INFO(
        get_logger(), "active tool restored from %s: %s (tool file %d)",
        tooling_state_path_.c_str(), t.name.c_str(), t.number);
    } else if (std::filesystem::exists(tooling_state_path_)) {
      RCLCPP_WARN(get_logger(), "ignoring %s: %s", tooling_state_path_.c_str(), err.c_str());
    }
  }
  setActiveTool(t);
  RCLCPP_INFO(
    get_logger(), "active tool: %s (tool file %d, %s) — every pose is expressed in this tool "
    "file unless the goal names another", t.name.c_str(), t.number, t.source.c_str());
}

void CobotControlNode::onSetActiveTool(
  cobot_interfaces::srv::SetActiveTool::Request::ConstSharedPtr req,
  cobot_interfaces::srv::SetActiveTool::Response::SharedPtr res)
{
  auto reject = [&](int32_t code, const std::string & m) {
      const ActiveTool cur = activeTool();
      res->success = false;
      res->status_code = code;
      res->message = m;
      res->tool_number = cur.number;
      res->name = cur.name;
      res->verified = cur.verified;
      RCLCPP_WARN(get_logger(), "set_active_tool rejected: %s", m.c_str());
    };

  if (busy_) {reject(3, "a motion goal is executing — the tool cannot change mid-move"); return;}

  std::string name = req->name;
  int32_t number = req->tool_number;
  bool configured = false;
  std::string configured_list;
  {
    std::lock_guard<std::mutex> lock(setup_mtx_);
    for (const auto & b : setup_.tools) {
      if (b.name == "none") {continue;}
      configured_list += (configured_list.empty() ? "" : ", ") + b.name;
    }
    if (!name.empty()) {
      if (name == "none") {
        number = 0;
        configured = true;
      } else if (const ToolBinding * b = setup_.findTool(name)) {
        number = b->tool_number;
        configured = true;
      } else {
        reject(2, "unknown tool name '" + name + "' (cobot_setup.yaml tools: none, " +
          configured_list + ")");
        return;
      }
    } else {
      if (number < 0 || number > 63) {reject(2, "name or tool_number 0-63 required"); return;}
      if (const ToolBinding * b = setup_.findToolByNumber(number)) {
        name = b->name;
        configured = true;
      } else if (number == 0) {
        name = "none";
        configured = true;
      }
    }
  }

  bool loaded = false, verified = false, identity = false;
  std::string cnd_name;
  {
    std::lock_guard<std::mutex> lock(profile_mtx_);
    loaded = profile_.loaded && !profile_.tools.empty();
    if (const ToolFileData * tf = profile_.findTool(number)) {
      verified = true;
      identity = tf->isIdentity();
      cnd_name = tf->name;
    }
  }
  if (name.empty()) {name = !cnd_name.empty() ? cnd_name : "tool" + std::to_string(number);}

  if (loaded && !verified && !req->force) {
    reject(4, "tool file " + std::to_string(number) +
      " is not in the controller's TOOL.CND (set force to override)");
    return;
  }
  if (!loaded && !configured) {
    reject(4, "controller profile not loaded: only 'none' and the tools configured in "
      "cobot_setup.yaml (" + configured_list + ") can be selected");
    return;
  }

  ActiveTool t;
  t.name = name;
  t.number = number;
  t.source = "service";
  t.verified = verified;
  setActiveTool(t);

  std::string msg = "active tool -> " + name + " (tool file " + std::to_string(number) + ")";
  if (identity && number != 0) {msg += " — WARNING: that tool file has an identity TCP";}
  if (!verified) {msg += " — not verified against TOOL.CND";}
  if (!configured) {msg += " — no cobot_setup.yaml tools entry (picks will be refused)";}
  int32_t code = 0;
  if (!tooling_state_path_.empty()) {
    std::string err;
    if (!ToolingState::save(tooling_state_path_, t, err)) {
      code = 5;
      msg += "; NOT persisted: " + err;
    }
  } else {
    msg += "; not persisted (runtime_dir unset)";
  }
  publishProfile();
  RCLCPP_WARN(get_logger(), "%s", msg.c_str());

  res->success = true;
  res->status_code = code;
  res->message = msg;
  res->tool_number = number;
  res->name = name;
  res->verified = verified;
}

void CobotControlNode::onConvertPose(
  cobot_interfaces::srv::ConvertPose::Request::ConstSharedPtr req,
  cobot_interfaces::srv::ConvertPose::Response::SharedPtr res)
{
  const int32_t active = active_tool_number_.load();
  const int32_t from = req->from_tool < 0 ? active : req->from_tool;
  const int32_t to = req->to_tool < 0 ? active : req->to_tool;
  res->from_tool = from;
  res->to_tool = to;
  res->x = req->x; res->y = req->y; res->z = req->z;
  res->rx = req->rx; res->ry = req->ry; res->rz = req->rz;
  if (from > 63 || to > 63) {fill(*res, RobotStatus{2, "tool numbers must be 0-63"}); return;}
  if (!req->joints_deg.empty() && req->joints_deg.size() != 6) {
    fill(*res, RobotStatus{2, "joints_deg must be empty or 6 values (S L U R B T)"});
    return;
  }
  if (!robot_->isConnected()) {fill(*res, RobotStatus::notConnected()); return;}

  const Pose in{req->x, req->y, req->z, req->rx, req->ry, req->rz};
  Pose out;
  std::vector<double> joints;
  const std::vector<double> * jin = req->joints_deg.empty() ? nullptr : &req->joints_deg;
  const RobotStatus st = robot_->convertPose(in, from, to, jin, out, joints);
  if (st.ok()) {
    res->x = out.x; res->y = out.y; res->z = out.z;
    res->rx = out.rx; res->ry = out.ry; res->rz = out.rz;
    res->joints_deg = joints;
  }
  fill(*res, st);
}

// ============================================================ controller profile

void CobotControlNode::startProfileRead()
{
  if (profile_source_ == "fixture") {return;}
  if (profile_loading_.exchange(true)) {return;}  // a read is already in flight
  joinProfileThread();                             // previous read finished
  std::lock_guard<std::mutex> lock(profile_thread_mtx_);
  profile_thread_ = std::thread(
    [this]() {
      readProfileFromController();
      profile_loading_ = false;
    });
}

void CobotControlNode::readProfileFromController()
{
  ProfileSource src;
  src.system_info = [this](std::string & model, std::string & version, std::string & err) {
      const RobotStatus st = robot_->readSystemInfo(model, version);
      if (!st.ok()) {err = st.text();}
      return st.ok();
    };
  src.list_cnd = [this](std::vector<std::string> & names, std::string & err) {
      const RobotStatus st = robot_->listConditionFiles(names);
      if (!st.ok()) {err = st.text();}
      return st.ok();
    };
  src.read_file = [this](const std::string & file, std::string & text, std::string & err) {
      const RobotStatus st = robot_->readControllerFile(file, text);
      if (!st.ok()) {
        err = st.text();
        int32_t rs7 = -1;
        if (robot_->readRsParam(7, rs7).ok()) {
          err += " (RS7=" + std::to_string(rs7) +
            "; set RS7=2 on the pendant to allow file reads outside REMOTE mode)";
        }
      }
      return st.ok();
    };

  RCLCPP_INFO(get_logger(), "reading controller profile (system info, %s, %s) — read-only",
    profile_files_.tool_file.c_str(), profile_files_.uframe_file.c_str());
  std::map<std::string, std::string> raw;
  ControllerProfileData data = buildProfile(src, profile_files_, &raw);
  if (profile_dump_) {dumpProfile(raw, data);}
  installProfile(std::move(data), "controller");
}

void CobotControlNode::installProfile(ControllerProfileData data, const std::string & source)
{
  data.source = source;
  {
    std::lock_guard<std::mutex> lock(profile_mtx_);
    profile_ = std::move(data);
    ++profile_read_count_;
  }
  // Re-verify the active tool against what the controller actually has.
  ActiveTool t = activeTool();
  bool verified = false, identity = false, loaded = false;
  size_t n_tools = 0, n_frames = 0, n_err = 0;
  std::string model, version;
  std::vector<std::string> errors;
  {
    std::lock_guard<std::mutex> lock(profile_mtx_);
    loaded = profile_.loaded;
    n_tools = profile_.tools.size();
    n_frames = profile_.user_frames.size();
    n_err = profile_.errors.size();
    errors = profile_.errors;
    model = profile_.model_name;
    version = profile_.software_version;
    if (const ToolFileData * tf = profile_.findTool(t.number)) {
      verified = true;
      identity = tf->isIdentity();
    }
  }
  {
    std::lock_guard<std::mutex> lock(tool_mtx_);
    active_tool_.verified = verified;
  }
  RCLCPP_INFO(
    get_logger(), "controller profile (%s): loaded=%d model='%s' version='%s' tools=%zu "
    "user_frames=%zu errors=%zu", source.c_str(), loaded, model.c_str(), version.c_str(),
    n_tools, n_frames, n_err);
  for (const auto & e : errors) {RCLCPP_WARN(get_logger(), "profile: %s", e.c_str());}
  if (loaded && n_tools > 0 && !verified) {
    RCLCPP_WARN(
      get_logger(), "active tool '%s' (tool file %d) is NOT in the controller's TOOL.CND",
      t.name.c_str(), t.number);
  } else if (identity && t.number != 0) {
    RCLCPP_WARN(
      get_logger(), "active tool '%s' (tool file %d) has an identity TCP on the controller",
      t.name.c_str(), t.number);
  }
  publishProfile();
}

void CobotControlNode::dumpProfile(
  const std::map<std::string, std::string> & raw, const ControllerProfileData & p)
{
  if (runtime_dir_.empty()) {return;}
  try {
    const std::filesystem::path dir = std::filesystem::path(runtime_dir_) / "controller_profile";
    std::filesystem::create_directories(dir);
    for (const auto & kv : raw) {
      std::ofstream f(dir / kv.first, std::ios::trunc | std::ios::binary);
      f << kv.second;
    }
    {
      std::ofstream f(dir / "system_info.yaml", std::ios::trunc);
      f << "model_name: \"" << p.model_name << "\"\n"
        << "software_version: \"" << p.software_version << "\"\n"
        << "read_at: \"" << isoTimestamp(p.read_at) << "\"\n"
        << "robot_ip: \"" << robot_ip_ << "\"\n";
    }
    {
      std::ofstream f(dir / "errors.txt", std::ios::trunc);
      for (const auto & e : p.errors) {f << "error: " << e << "\n";}
      for (const auto & w : p.warnings) {f << "warning: " << w << "\n";}
    }
    RCLCPP_INFO(get_logger(), "controller profile dumped to %s", dir.string().c_str());
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "profile dump failed: %s", e.what());
  }
}

void CobotControlNode::publishProfile()
{
  if (!profile_pub_) {return;}
  ControllerProfile msg;
  ControllerProfileData p;
  uint32_t count = 0;
  {
    std::lock_guard<std::mutex> lock(profile_mtx_);
    p = profile_;
    count = profile_read_count_;
  }
  const auto ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(p.read_at.time_since_epoch()).count();
  msg.header.stamp = ns > 0 ? rclcpp::Time(ns, RCL_SYSTEM_TIME) : now();
  msg.read_count = count;
  msg.loaded = p.loaded;
  msg.source = p.source;
  msg.robot_ip = robot_ip_;
  msg.model_name = p.model_name;
  msg.software_version = p.software_version;
  for (const auto & t : p.tools) {
    cobot_interfaces::msg::ToolFile m;
    m.number = t.number;
    m.name = t.name;
    m.x = t.x; m.y = t.y; m.z = t.z; m.rx = t.rx; m.ry = t.ry; m.rz = t.rz;
    m.weight_kg = t.weight_kg;
    m.cog_mm = t.cog_mm;
    m.inertia = t.inertia;
    m.is_identity = t.isIdentity();
    for (const auto & kv : t.extra) {
      KeyValue e; e.key = kv.first; e.value = kv.second; m.extra.push_back(e);
    }
    msg.tools.push_back(m);
  }
  for (const auto & f : p.user_frames) {
    cobot_interfaces::msg::UserFrame m;
    m.number = f.number;
    m.name = f.name;
    m.tool_number = f.tool_number;
    m.group = f.group;
    m.pose = f.pose;
    m.defined = f.defined();
    for (const auto & kv : f.extra) {
      KeyValue e; e.key = kv.first; e.value = kv.second; m.extra.push_back(e);
    }
    msg.user_frames.push_back(m);
  }
  msg.cnd_files = p.cnd_files;
  msg.errors = p.errors;
  msg.warnings = p.warnings;
  if (!p.loaded && p.source == "none") {msg.errors.push_back("not connected");}
  {
    std::lock_guard<std::mutex> lock(setup_mtx_);
    for (const auto & b : setup_.tools) {
      msg.configured_tool_names.push_back(b.name);
      msg.configured_tool_numbers.push_back(b.tool_number);
    }
  }
  const ActiveTool t = activeTool();
  msg.active_tool_number = t.number;
  msg.active_tool_name = t.name;
  msg.active_tool_source = t.source;
  msg.active_tool_verified = t.verified;
  profile_pub_->publish(msg);
}

void CobotControlNode::onReloadProfile(
  Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res)
{
  if (profile_source_ == "fixture") {
    std::map<std::string, std::string> raw;
    installProfile(buildProfile(fixtureSource(profile_fixture_dir_), profile_files_, &raw), "fixture");
    res->success = true;
    res->message = "fixture profile reloaded from " + profile_fixture_dir_;
    return;
  }
  if (!robot_->isConnected()) {
    res->success = false;
    res->message = "not connected";
    return;
  }
  if (busy_) {
    res->success = false;
    res->message = "a motion goal is executing — reload the profile when idle";
    return;
  }
  if (profile_loading_.exchange(true)) {
    res->success = false;
    res->message = "profile read already in progress";
    return;
  }
  readProfileFromController();  // synchronous; reentrant group keeps the poll alive
  profile_loading_ = false;
  std::lock_guard<std::mutex> lock(profile_mtx_);
  res->success = profile_.loaded;
  res->message = "profile read #" + std::to_string(profile_read_count_) + ": " +
    std::to_string(profile_.tools.size()) + " tool file(s), " +
    std::to_string(profile_.user_frames.size()) + " user frame(s), " +
    std::to_string(profile_.errors.size()) + " error(s)";
  for (const auto & e : profile_.errors) {res->message += "; " + e;}
}

}  // namespace cobot_control
