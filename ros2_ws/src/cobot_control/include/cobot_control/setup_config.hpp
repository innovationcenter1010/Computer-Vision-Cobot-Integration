#pragma once
// Loader for cobot_setup.yaml (the `io` and `motion_profiles` sections; the
// `tools` section belongs to cobot_operation). Plain yaml-cpp — ROS params
// cannot express a map of named structs.

#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "cobot_control/motoman_robot.hpp"

namespace cobot_control
{

struct IoAddress
{
  std::string name;
  IoKind kind{IoKind::GeneralInput};
  uint16_t group{0};
  uint8_t bit{0};
};

struct SetupConfig
{
  std::vector<IoAddress> sensors;
  std::vector<IoAddress> outputs;
  std::map<std::string, MotionProfile> motion_profiles;  // always contains "default"

  // Throws std::runtime_error with a path-qualified message on any problem.
  static SetupConfig load(const std::string & path);

  const IoAddress * findSensor(const std::string & name) const;
  const IoAddress * findOutput(const std::string & name) const;
};

const char * ioKindName(IoKind k);
bool parseIoKind(const std::string & s, IoKind & out);

}  // namespace cobot_control
