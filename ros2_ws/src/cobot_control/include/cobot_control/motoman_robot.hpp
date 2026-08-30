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

enum class Interp { Joint, Linear };
enum class IoKind { GeneralInput, GeneralOutput };

struct MotionProfile
{
  Interp interpolation{Interp::Joint};
  double speed{25.0};        // joint: percent (0-100); linear: mm/s (0-1500)
  double accel_ratio{25.0};  // 20-100 %
  double decel_ratio{25.0};  // 20-100 %
  int32_t accuracy_mm{50};   // -1 = planner rounds corners; 0 = exact
  int32_t tool_number{0};    // YRC1000 tool file
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
  bool reached{false};
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
  RobotStatus activeAlarms(std::vector<Alarm> & out);
  RobotStatus clearFaults();
  RobotStatus setServos(bool on);
  RobotStatus setHold(bool on);
  RobotStatus pendantMessage(const std::string & text);  // truncated to 30 chars

  // ---- motion ----
  RobotStatus addPoint(const Pose & pose, const MotionProfile & profile);
  RobotStatus motionStart();                      // starts, or RESUMES after motionStop(false)
  RobotStatus motionStop(bool clear_trajectory);  // immediate controlled stop
  RobotStatus clearTrajectory();
  RobotStatus motionProgress(MotionProgress & out);
  // IK check against the current joint configuration. A failed IK is
  // reported as reachable=false with `why`, not as an error status.
  RobotStatus isReachable(
    const Pose & pose, int32_t tool_number, bool & reachable, std::string & why);

  // ---- variables ----
  RobotStatus readInt(uint16_t index, int32_t & out);
  RobotStatus writeInt(uint16_t index, int32_t value);
  RobotStatus readPositionVar(uint16_t index, Pose & out);
  RobotStatus writePositionVar(uint16_t index, const Pose & pose);

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
