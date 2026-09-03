#pragma once
// Thread-safe wrapper over the YMConnect SDK. NO ROS in this file: the node
// owns timers and threads; this class owns exactly one controller pointer
// and the mutex that serialises every SDK call on it.
//
// Evolved (with git history) from motoman_cpp/src/MotomanRobot.hpp, the
// pybind-era wrapper. Differences that matter:
//   * no watchdog thread, no std::any payloads, no std::cout
//   * every call goes through withController(): lock + null check
//   * connect() checks the OpenConnection result before touching it
//   * IO reads say which IOType they read (the old read_general_output_bit
//     read GeneralInput)
//   * joint vs linear interpolation via the SDK JointMotion/LinearMotion
//     structs, so the speed unit always matches the SpeedType

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "YMConnect.h"

namespace cobot_control
{

struct RobotStatus
{
  int32_t code{0};      // 0 ok; 1 not connected (synthetic); else YMConnect StatusCode
  std::string message;

  bool ok() const { return code == 0; }
  std::string text() const;  // "[code] message", or just message when ok

  static RobotStatus okay(const std::string & m = "ok") { return {0, m}; }
  static RobotStatus notConnected() { return {1, "not connected"}; }
  static RobotStatus from(const StatusInfo & s) { return {s.StatusCode, s.Message}; }
};

struct Pose
{
  double x{0}, y{0}, z{0}, rx{0}, ry{0}, rz{0};
  std::string str() const;
};

// Wraps a degree value into [-180, 180), the wrist (T-axis) Euler convention.
// Used only for rz after an offset is added — never for rx/ry (fixed by pick
// orientation) or the S/L/U/R/B joints, which have no wraparound.
double normalizeDeg180(double deg);

enum class Interp { Joint, Linear };
enum class IoKind { GeneralInput, GeneralOutput };

struct MotionProfile
{
  Interp interpolation{Interp::Joint};
  double speed{25.0};        // joint: percent (0-100); linear: mm/s (0-1500)
  double accel_ratio{25.0};  // 20-100 %
  double decel_ratio{25.0};  // 20-100 %
  int32_t accuracy_mm{50};   // -1 = planner rounds corners; 0 = exact
  // No tool_number here on purpose: a profile says HOW to move, the tool
  // file (TCP) is the node-level active tool, passed explicitly to addPoint.
};

struct ControllerState
{
  bool servo_on{false}, running{false}, in_hold{false}, alarming{false}, erroring{false};
  int control_mode{0};  // 0 teach, 1 play, 2 remote (ControllerStateData::ControlMode order)
};

struct Alarm
{
  uint32_t code{0}, subcode{0};
  std::string name, time;
};

struct MotionProgress
{
  double percent{0};
  int32_t steps_remaining{0};
  // No `reached` flag on purpose: arrival is the NODE's judgement
  // ("buffer empty AND controller stopped", after motion was observed to
  // start). The old "percent >= 99 && steps <= 1" heuristic reported
  // arrival while decelerating and accepted a whole unexecuted point.
};

struct ConnectOptions
{
  uint32_t timeout_ms{500};  // YMConnect receive timeout per packet
  uint32_t retries{3};       // YMConnect send retries
  bool logging{false};       // SDK writes YMConnectLog.txt next to the .so when true
};

class MotomanRobot
{
public:
  explicit MotomanRobot(std::string ip, ConnectOptions opts = ConnectOptions());
  ~MotomanRobot();
  MotomanRobot(const MotomanRobot &) = delete;
  MotomanRobot & operator=(const MotomanRobot &) = delete;

  // OpenConnection -> SetServos(ON) -> ClearAllTrajectory. One attempt; the
  // caller retries. Blocks up to ~timeout_ms * retries when the robot is off.
  RobotStatus connect();
  // servos_off=false skips the SetServos(OFF) round-trip (use when the link
  // is already known to be dead, so we do not wait on a timeout).
  RobotStatus disconnect(bool servos_off = true);
  bool isConnected() const { return connected_.load(); }
  const std::string & ip() const { return ip_; }

  // ---- state / IO ----
  RobotStatus readState(ControllerState & out);
  RobotStatus readIoByte(IoKind kind, uint16_t group, uint8_t & out);
  RobotStatus readIoBit(IoKind kind, uint16_t group, uint8_t bit, bool & out);
  RobotStatus writeOutputBit(uint16_t group, uint8_t bit, bool value);
  RobotStatus readCurrentPose(int32_t tool_number, Pose & out);
  // Joint angles S L U R B T in degrees (ReadPositionData JointDegrees).
  RobotStatus readCurrentJoints(std::vector<double> & joints_deg);
  // Pose + joints back-to-back under ONE lock (teach / remember flows).
  RobotStatus readCurrentPoseAndJoints(
    int32_t tool_number, Pose & pose, std::vector<double> & joints_deg);

  // ---- read-only controller profile (system info, files) ----
  RobotStatus readSystemInfo(std::string & model_name, std::string & software_version);
  RobotStatus listConditionFiles(std::vector<std::string> & names);
  // SaveFromControllerToString: the controller file is copied INTO `text`;
  // nothing is written to the controller. Needs RS7=2 outside REMOTE mode.
  RobotStatus readControllerFile(const std::string & file_name, std::string & text);
  RobotStatus readRsParam(int32_t number, int32_t & value);  // RS<number>
  RobotStatus activeAlarms(std::vector<Alarm> & out);
  RobotStatus clearFaults();
  RobotStatus setServos(bool on);
  RobotStatus setHold(bool on);
  RobotStatus pendantMessage(const std::string & text);  // truncated to 30 chars

  // ---- motion ----
  RobotStatus addPoint(const Pose & pose, const MotionProfile & profile, int32_t tool_number);
  RobotStatus motionStart();                      // starts, or RESUMES after motionStop(false)
  RobotStatus motionStop(bool clear_trajectory);  // immediate controlled stop
  RobotStatus clearTrajectory();
  RobotStatus motionProgress(MotionProgress & out);
  // IK check against the current joint configuration (the controller's own
  // solver — the same one the motion planner uses, so a "no solution" here is
  // exactly what would later become interpolation alarm 4684). A failed IK is
  // reported as reachable=false with `why`, not as an error status. On
  // success `joints_deg_out` holds the solution (S L U R B T, degrees) so the
  // caller can run a joint-limit margin check.
  RobotStatus isReachable(
    const Pose & pose, int32_t tool_number, bool & reachable, std::string & why,
    std::vector<double> & joints_deg_out);
  // Same physical configuration, re-expressed through another tool file,
  // using the CONTROLLER's kinematics (no motion, nothing written):
  //   cart(from_tool) --IK (Delta, seeded with the current joints)--> joints
  //   --FK with toolNumber = to_tool--> cart(to_tool)
  // joints_in (6 values) skips the IK step. joints_out = the joints the
  // returned pose corresponds to.
  RobotStatus convertPose(
    const Pose & in, int32_t from_tool, int32_t to_tool, const std::vector<double> * joints_in,
    Pose & out, std::vector<double> & joints_out);

  // ---- variables ----
  RobotStatus readInt(uint16_t index, int32_t & out);
  RobotStatus writeInt(uint16_t index, int32_t value);
  RobotStatus readPositionVar(uint16_t index, Pose & out, int32_t & tool_number);
  RobotStatus writePositionVar(uint16_t index, const Pose & pose, int32_t tool_number);

private:
  template<class F>
  RobotStatus withController(F && f);

  std::string ip_;
  ConnectOptions opts_;
  mutable std::mutex mtx_;
  MotomanController * controller_{nullptr};
  std::atomic<bool> connected_{false};
};

}  // namespace cobot_control
