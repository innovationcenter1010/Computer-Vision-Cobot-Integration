#pragma once
// cobot_control — the ONLY process that talks to the Motoman (YMConnect).
// Hardware primitives as ROS interfaces, no choreography, no task state.
//
//   action  /cobot/move_to_pose      one supervised move; feedback progress;
//                                    cancel = immediate MotionStop(true)
//   srv     /cobot/set_output_bit    write a named / raw output bit
//   srv     /cobot/read_io_bit       one-off IO read (poll is the normal path)
//   srv     /cobot/motion_stop       immediate controlled stop
//   srv     /cobot/pause /resume     hold the arm in place / continue
//   srv     /cobot/clear_trajectory, /clear_faults, /servo_on, /servo_off,
//           /reconnect               std_srvs/Trigger
//   srv     /cobot/set_motion_profile, /read_int_var, /write_int_var,
//           /read_position_var, /write_position_var, /get_current_pose,
//           /get_active_alarms
//   srv     /cobot/set_active_tool   select the tool file (TCP) poses are
//                                    expressed in; persisted in runtime_dir
//   srv     /cobot/convert_pose      cart(tool A) -> joints -> cart(tool B)
//                                    via the controller kinematics (no motion)
//   srv     /cobot/reload_profile    re-read the controller profile (Trigger)
//   topic   /cobot/state  (CobotState, poll rate)   everything read from the
//                                                   controller, by name
//   topic   /cobot/status (CobotStatus, latched)    disconnected/idle/busy/error
//   topic   /cobot/profile (ControllerProfile, latched)  system info, tool
//                          files, user frames — read ONCE per connect, READ-ONLY
//   topic   /health/heartbeat
//
// Threading: MultiThreadedExecutor. The poll timer (exclusive group) is the
// only periodic talker; the move loop runs in its own std::thread and waits
// on the poll's condition variable instead of polling the controller itself;
// services and action callbacks are reentrant so pause / stop / reads stay
// serviceable mid-move. The profile read (multi-packet file transfers) runs
// in its own one-shot thread after connect so the poll is not frozen. Every
// SDK call serialises on MotomanRobot's mutex.

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "cobot_interfaces/action/move_to_pose.hpp"
#include "cobot_interfaces/msg/cobot_state.hpp"
#include "cobot_interfaces/msg/cobot_status.hpp"
#include "cobot_interfaces/msg/controller_profile.hpp"
#include "cobot_interfaces/msg/heartbeat.hpp"
#include "cobot_interfaces/srv/convert_pose.hpp"
#include "cobot_interfaces/srv/get_active_alarms.hpp"
#include "cobot_interfaces/srv/get_current_pose.hpp"
#include "cobot_interfaces/srv/motion_stop.hpp"
#include "cobot_interfaces/srv/read_int_var.hpp"
#include "cobot_interfaces/srv/read_io_bit.hpp"
#include "cobot_interfaces/srv/read_position_var.hpp"
#include "cobot_interfaces/srv/set_active_tool.hpp"
#include "cobot_interfaces/srv/set_motion_profile.hpp"
#include "cobot_interfaces/srv/set_output_bit.hpp"
#include "cobot_interfaces/srv/write_int_var.hpp"
#include "cobot_interfaces/srv/write_position_var.hpp"

#include "cobot_control/controller_profile.hpp"
#include "cobot_control/motoman_robot.hpp"
#include "cobot_control/setup_config.hpp"
#include "cobot_control/tooling_state.hpp"

namespace cobot_control
{

class CobotControlNode : public rclcpp::Node
{
public:
  explicit CobotControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CobotControlNode() override;

private:
  using MoveToPose = cobot_interfaces::action::MoveToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveToPose>;
  using CobotState = cobot_interfaces::msg::CobotState;
  using CobotStatus = cobot_interfaces::msg::CobotStatus;
  using ControllerProfile = cobot_interfaces::msg::ControllerProfile;
  using Trigger = std_srvs::srv::Trigger;

  // ---- poll / connection supervision ----
  void pollTick();
  bool tryConnect();
  void onLinkLost(const std::string & why);
  void publishState(const CobotState & msg);
  void publishStatus(uint8_t state, const std::string & detail, bool force = false);
  void heartbeatTick();

  // ---- controller profile (read once per connect, read-only) ----
  void startProfileRead();            // one-shot thread; idempotent
  void readProfileFromController();   // body of that thread / reload service
  void installProfile(ControllerProfileData data, const std::string & source);
  void dumpProfile(const std::map<std::string, std::string> & raw, const ControllerProfileData & p);
  void publishProfile();
  void joinProfileThread();

  // ---- active tool ----
  void initActiveTool();
  ActiveTool activeTool() const;
  void setActiveTool(const ActiveTool & tool);

  // ---- MoveToPose ----
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid, MoveToPose::Goal::ConstSharedPtr goal);
  rclcpp_action::CancelResponse handleCancel(std::shared_ptr<GoalHandle> gh);
  void handleAccepted(std::shared_ptr<GoalHandle> gh);
  void executeMove(std::shared_ptr<GoalHandle> gh);
  MotionProfile resolveProfile(
    const MoveToPose::Goal & goal, std::string & profile_name, int32_t & tool_number,
    double & mount_yaw_offset_deg);
  void holdMotion();     // pause: MotionStop(false) or SetHold(ON)
  void releaseMotion();  // resume: MotionStart() or SetHold(OFF)
  void joinMoveThread();
  // Margin check against joint_limits_min/max_deg (S L U R B T). Empty
  // limits = check skipped (rely on the controller's IK alone).
  bool jointLimitsOk(const std::vector<double> & joints_deg, std::string & detail) const;

  // ---- services ----
  bool resolveIo(
    const std::string & name, uint8_t io_type, uint16_t group, uint8_t bit, bool want_output,
    IoAddress & out, std::string & err) const;

  void onSetOutputBit(
    cobot_interfaces::srv::SetOutputBit::Request::ConstSharedPtr req,
    cobot_interfaces::srv::SetOutputBit::Response::SharedPtr res);
  void onReadIoBit(
    cobot_interfaces::srv::ReadIoBit::Request::ConstSharedPtr req,
    cobot_interfaces::srv::ReadIoBit::Response::SharedPtr res);
  void onMotionStop(
    cobot_interfaces::srv::MotionStop::Request::ConstSharedPtr req,
    cobot_interfaces::srv::MotionStop::Response::SharedPtr res);
  void onSetMotionProfile(
    cobot_interfaces::srv::SetMotionProfile::Request::ConstSharedPtr req,
    cobot_interfaces::srv::SetMotionProfile::Response::SharedPtr res);
  void onReadIntVar(
    cobot_interfaces::srv::ReadIntVar::Request::ConstSharedPtr req,
    cobot_interfaces::srv::ReadIntVar::Response::SharedPtr res);
  void onWriteIntVar(
    cobot_interfaces::srv::WriteIntVar::Request::ConstSharedPtr req,
    cobot_interfaces::srv::WriteIntVar::Response::SharedPtr res);
  void onReadPositionVar(
    cobot_interfaces::srv::ReadPositionVar::Request::ConstSharedPtr req,
    cobot_interfaces::srv::ReadPositionVar::Response::SharedPtr res);
  void onWritePositionVar(
    cobot_interfaces::srv::WritePositionVar::Request::ConstSharedPtr req,
    cobot_interfaces::srv::WritePositionVar::Response::SharedPtr res);
  void onGetCurrentPose(
    cobot_interfaces::srv::GetCurrentPose::Request::ConstSharedPtr req,
    cobot_interfaces::srv::GetCurrentPose::Response::SharedPtr res);
  void onGetActiveAlarms(
    cobot_interfaces::srv::GetActiveAlarms::Request::ConstSharedPtr req,
    cobot_interfaces::srv::GetActiveAlarms::Response::SharedPtr res);
  void onSetActiveTool(
    cobot_interfaces::srv::SetActiveTool::Request::ConstSharedPtr req,
    cobot_interfaces::srv::SetActiveTool::Response::SharedPtr res);
  void onConvertPose(
    cobot_interfaces::srv::ConvertPose::Request::ConstSharedPtr req,
    cobot_interfaces::srv::ConvertPose::Response::SharedPtr res);
  void onReloadProfile(Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res);
  void onPause(Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res);
  void onResume(Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res);
  void onClearTrajectory(Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res);
  void onClearFaults(Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res);
  void onServoOn(Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res);
  void onServoOff(Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res);
  void onReconnect(Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res);

  // ---- parameters ----
  std::string robot_ip_;
  std::string setup_file_;
  std::string runtime_dir_;         // tooling_state.yaml + controller_profile/ dumps
  std::string profile_source_;      // "controller" | "fixture"
  std::string profile_fixture_dir_;
  ProfileFiles profile_files_;
  bool profile_dump_{true};
  bool poll_joints_{true};
  double poll_period_s_{0.1};
  double connect_retry_period_s_{1.0};
  double motion_timeout_s_{100.0};
  bool pause_uses_hold_{false};
  bool check_reachability_{true};
  bool pendant_messages_{true};
  double heartbeat_period_s_{2.0};
  // Motion wait (ported from the fork's hardware-tested cobot_driver).
  double motion_poll_period_s_{0.05};
  double motion_start_grace_s_{3.0};
  double motion_feedback_hz_{5.0};
  // Joint limits are live-tunable (dev panel → set_parameters): the callback
  // below rewrites them under joint_limits_mtx_ while motions read a copy.
  double joint_limit_margin_deg_{5.0};
  std::vector<double> joint_limits_min_deg_;
  std::vector<double> joint_limits_max_deg_;
  mutable std::mutex joint_limits_mtx_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // ---- hardware ----
  std::unique_ptr<MotomanRobot> robot_;
  SetupConfig setup_;
  mutable std::mutex setup_mtx_;  // motion_profiles are editable at runtime

  // ---- active tool (the coordinate system poses are expressed in) ----
  ActiveTool active_tool_;
  mutable std::mutex tool_mtx_;
  std::atomic<int32_t> active_tool_number_{0};  // lock-free mirror for the hot paths
  std::string tooling_state_path_;

  // ---- controller profile ----
  ControllerProfileData profile_;
  mutable std::mutex profile_mtx_;
  uint32_t profile_read_count_{0};
  std::atomic<bool> profile_loading_{false};
  std::thread profile_thread_;
  std::mutex profile_thread_mtx_;

  // ---- callback groups / timers ----
  rclcpp::CallbackGroup::SharedPtr poll_group_;
  rclcpp::CallbackGroup::SharedPtr hb_group_;
  rclcpp::CallbackGroup::SharedPtr io_group_;  // reentrant: services + action
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr hb_timer_;
  rclcpp::Time last_connect_attempt_;
  int connect_attempts_{0};

  // ---- poll snapshot shared with the move loop ----
  std::mutex state_mtx_;
  std::condition_variable poll_cv_;
  uint64_t poll_seq_{0};
  CobotState last_state_;
  // Motion progress is read fresh by the move loop itself; the poll does not
  // cache it (a stale sample once produced a false "reached pose").

  // ---- motion state ----
  std::atomic<bool> busy_{false};
  std::atomic<bool> paused_{false};          // pause latch (/cobot/pause)
  std::atomic<bool> held_{false};            // arm currently held by pause
  std::atomic<bool> stop_requested_{false};  // /cobot/motion_stop while busy
  std::thread move_thread_;
  std::mutex move_thread_mtx_;

  // ---- publishers ----
  rclcpp::Publisher<CobotStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<CobotState>::SharedPtr state_pub_;
  rclcpp::Publisher<ControllerProfile>::SharedPtr profile_pub_;
  rclcpp::Publisher<cobot_interfaces::msg::Heartbeat>::SharedPtr hb_pub_;
  std::mutex status_mtx_;
  uint8_t last_status_state_{255};

  // ---- servers ----
  rclcpp_action::Server<MoveToPose>::SharedPtr move_server_;
  rclcpp::Service<cobot_interfaces::srv::SetOutputBit>::SharedPtr set_output_bit_srv_;
  rclcpp::Service<cobot_interfaces::srv::ReadIoBit>::SharedPtr read_io_bit_srv_;
  rclcpp::Service<cobot_interfaces::srv::MotionStop>::SharedPtr motion_stop_srv_;
  rclcpp::Service<cobot_interfaces::srv::SetMotionProfile>::SharedPtr set_motion_profile_srv_;
  rclcpp::Service<cobot_interfaces::srv::ReadIntVar>::SharedPtr read_int_var_srv_;
  rclcpp::Service<cobot_interfaces::srv::WriteIntVar>::SharedPtr write_int_var_srv_;
  rclcpp::Service<cobot_interfaces::srv::ReadPositionVar>::SharedPtr read_position_var_srv_;
  rclcpp::Service<cobot_interfaces::srv::WritePositionVar>::SharedPtr write_position_var_srv_;
  rclcpp::Service<cobot_interfaces::srv::GetCurrentPose>::SharedPtr get_current_pose_srv_;
  rclcpp::Service<cobot_interfaces::srv::GetActiveAlarms>::SharedPtr get_active_alarms_srv_;
  rclcpp::Service<cobot_interfaces::srv::SetActiveTool>::SharedPtr set_active_tool_srv_;
  rclcpp::Service<cobot_interfaces::srv::ConvertPose>::SharedPtr convert_pose_srv_;
  rclcpp::Service<Trigger>::SharedPtr reload_profile_srv_;
  rclcpp::Service<Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<Trigger>::SharedPtr resume_srv_;
  rclcpp::Service<Trigger>::SharedPtr clear_trajectory_srv_;
  rclcpp::Service<Trigger>::SharedPtr clear_faults_srv_;
  rclcpp::Service<Trigger>::SharedPtr servo_on_srv_;
  rclcpp::Service<Trigger>::SharedPtr servo_off_srv_;
  rclcpp::Service<Trigger>::SharedPtr reconnect_srv_;
};

}  // namespace cobot_control
