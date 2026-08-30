#include "cobot_control/cobot_control_node.hpp"

#include <algorithm>
#include <chrono>
#include <map>
#include <utility>

#include "cobot_interfaces/msg/alarm.hpp"
#include "cobot_interfaces/msg/key_value.hpp"

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

  if (setup_file_.empty()) {
    throw std::runtime_error(
            "setup_file parameter is required (cobot_bringup/config/cobot_setup.yaml)");
  }
  setup_ = SetupConfig::load(setup_file_);  // throws with a clear message; launch respawns
  pose_tool_number_ = setup_.motion_profiles.at("default").tool_number;
  RCLCPP_INFO(
    get_logger(), "setup: %zu sensors, %zu outputs, %zu motion profiles from %s",
    setup_.sensors.size(), setup_.outputs.size(), setup_.motion_profiles.size(),
    setup_file_.c_str());

  ConnectOptions copts;
  copts.timeout_ms = static_cast<uint32_t>(std::max(connect_timeout_ms, 50));
  copts.retries = static_cast<uint32_t>(std::max(connect_retries, 1));
  copts.logging = ymconnect_logging;
  robot_ = std::make_unique<MotomanRobot>(robot_ip_, copts);

  // ---- publishers ----
  status_pub_ = create_publisher<CobotStatus>(
    "/cobot/status", rclcpp::QoS(1).reliable().transient_local());
  state_pub_ = create_publisher<CobotState>("/cobot/state", rclcpp::QoS(1).reliable());
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

  RCLCPP_INFO(get_logger(), "cobot_control up; polling %s every %.0f ms", robot_ip_.c_str(),
    poll_period_s_ * 1000.0);
}

CobotControlNode::~CobotControlNode()
{
  if (poll_timer_) {poll_timer_->cancel();}
  if (hb_timer_) {hb_timer_->cancel();}
  if (busy_) {robot_->motionStop(true);}
  joinMoveThread();
  robot_.reset();  // disconnect (servos off)
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

  Pose p;
  if (robot_->readCurrentPose(pose_tool_number_, p).ok()) {
    msg.x = p.x; msg.y = p.y; msg.z = p.z;
    msg.rx = p.rx; msg.ry = p.ry; msg.rz = p.rz;
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

  if (busy_) {
    MotionProgress mp;
    const RobotStatus ps = robot_->motionProgress(mp);
    std::lock_guard<std::mutex> lock(state_mtx_);
    last_progress_ = mp;
    last_progress_status_ = ps;
  } else {
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
  hb_pub_->publish(hb);
}

// ============================================================ MoveToPose

MotionProfile CobotControlNode::resolveProfile(
  const MoveToPose::Goal & goal, std::string & profile_name)
{
  std::lock_guard<std::mutex> lock(setup_mtx_);
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
  if (goal.tool_number >= 0) {p.tool_number = goal.tool_number;}
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

void CobotControlNode::executeMove(std::shared_ptr<GoalHandle> gh)
{
  const auto goal = gh->get_goal();
  auto result = std::make_shared<MoveToPose::Result>();
  auto feedback = std::make_shared<MoveToPose::Feedback>();
  const Pose pose{goal->x, goal->y, goal->z, goal->rx, goal->ry, goal->rz};
  std::string profile_name;
  const MotionProfile prof = resolveProfile(*goal, profile_name);

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
    const RobotStatus st = robot_->isReachable(pose, prof.tool_number, reachable, why);
    if (!st.ok()) {abort(st.code, "reachability check: " + st.text()); finish(); return;}
    if (!reachable) {abort(2, "unreachable pose: " + why); finish(); return;}
  }

  RobotStatus st = robot_->addPoint(pose, prof);
  if (!st.ok()) {abort(st.code, "add point: " + st.text()); finish(); return;}
  st = robot_->motionStart();
  if (!st.ok()) {
    robot_->clearTrajectory();
    abort(st.code, "motion start: " + st.text());
    finish();
    return;
  }
  RCLCPP_INFO(
    get_logger(), "moving to %s [%s %s speed=%.1f]", pose.str().c_str(), profile_name.c_str(),
    prof.interpolation == Interp::Linear ? "linear" : "joint", prof.speed);

  uint64_t seen_seq;
  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    seen_seq = poll_seq_;
  }
  // The poll may have sampled progress just before MotionStart; require two
  // fresh samples before trusting `reached`.
  int fresh_samples = 0;
  auto segment_start = std::chrono::steady_clock::now();
  const auto wait_step = std::chrono::duration<double>(std::max(poll_period_s_, 0.02) * 2.0);

  while (rclcpp::ok()) {
    CobotState snapshot;
    MotionProgress prog;
    RobotStatus prog_st;
    {
      std::unique_lock<std::mutex> lock(state_mtx_);
      if (poll_cv_.wait_for(lock, wait_step, [&] {return poll_seq_ != seen_seq;})) {
        seen_seq = poll_seq_;
        ++fresh_samples;
      }
      snapshot = last_state_;
      prog = last_progress_;
      prog_st = last_progress_status_;
    }

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
    if (!robot_->isConnected() || !snapshot.connected) {
      abort(1, "robot link lost during motion");
      break;
    }
    if (snapshot.alarming) {
      robot_->motionStop(true);
      robot_->clearTrajectory();
      abort(4, "controller alarm: " + alarmText(snapshot.alarms));
      break;
    }

    if (paused_) {
      holdMotion();
      feedback->paused = true;
      feedback->progress_percent = static_cast<float>(prog.percent);
      feedback->steps_remaining = prog.steps_remaining;
      gh->publish_feedback(feedback);
      segment_start = std::chrono::steady_clock::now();  // timeout counts moving time only
      continue;
    }
    if (held_) {
      releaseMotion();
      fresh_samples = 0;
      continue;
    }

    if (fresh_samples < 2) {continue;}

    if (!prog_st.ok()) {
      robot_->motionStop(true);
      abort(prog_st.code, "progress read: " + prog_st.text());
      break;
    }
    feedback->paused = false;
    feedback->progress_percent = static_cast<float>(prog.percent);
    feedback->steps_remaining = prog.steps_remaining;
    gh->publish_feedback(feedback);

    if (prog.reached) {
      result->success = true;
      result->status_code = 0;
      result->message = "reached pose";
      gh->succeed(result);
      RCLCPP_INFO(get_logger(), "reached %s", pose.str().c_str());
      break;
    }

    const double moving_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - segment_start).count();
    if (moving_s > motion_timeout_s_) {
      robot_->motionStop(true);
      robot_->clearTrajectory();
      abort(5, "motion timeout after " + std::to_string(static_cast<int>(moving_s)) + " s");
      break;
    }
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
    get_logger(), "motion profile '%s': %s speed=%.1f accel=%.0f decel=%.0f acc=%d tool=%d",
    name.c_str(), p.interpolation == Interp::Linear ? "linear" : "joint", p.speed, p.accel_ratio,
    p.decel_ratio, p.accuracy_mm, p.tool_number);
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
  const RobotStatus st = robot_->readPositionVar(req->index, p);
  res->x = p.x; res->y = p.y; res->z = p.z; res->rx = p.rx; res->ry = p.ry; res->rz = p.rz;
  fill(*res, st);
}

void CobotControlNode::onWritePositionVar(
  cobot_interfaces::srv::WritePositionVar::Request::ConstSharedPtr req,
  cobot_interfaces::srv::WritePositionVar::Response::SharedPtr res)
{
  const Pose p{req->x, req->y, req->z, req->rx, req->ry, req->rz};
  fill(*res, robot_->writePositionVar(req->index, p));
}

void CobotControlNode::onGetCurrentPose(
  cobot_interfaces::srv::GetCurrentPose::Request::ConstSharedPtr,
  cobot_interfaces::srv::GetCurrentPose::Response::SharedPtr res)
{
  Pose p;
  const RobotStatus st = robot_->readCurrentPose(pose_tool_number_, p);
  res->x = p.x; res->y = p.y; res->z = p.z; res->rx = p.rx; res->ry = p.ry; res->rz = p.rz;
  fill(*res, st, p.str());
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

}  // namespace cobot_control
