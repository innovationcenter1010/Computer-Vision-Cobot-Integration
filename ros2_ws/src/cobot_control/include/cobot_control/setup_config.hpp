#pragma once
// Loader for cobot_setup.yaml: the `io` and `motion_profiles` sections plus
// the name -> tool_number mapping of the `tools` section and the
// `active_tool` boot default. The per-tool CELL values (marker offset, pick
// heights, ...) in `tools` belong to cobot_operation / localization; only
// `tool_number` is read here. Plain yaml-cpp — ROS params cannot express a
// map of named structs.

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

// One `tools.<name>` entry as far as cobot_control cares: which YRC1000 tool
// file it means. "none" is reserved for tool file 0 (the flange / default
// TCP) and may not be declared with another number.
struct ToolBinding
{
  std::string name;
  int32_t tool_number{0};
};

struct SetupConfig
{
  std::vector<IoAddress> sensors;
  std::vector<IoAddress> outputs;
  std::map<std::string, MotionProfile> motion_profiles;  // always contains "default"
  std::vector<ToolBinding> tools;                        // may be empty
  std::string active_tool_default{"none"};               // "none" or a tools key
  // Constant added to every commanded MoveToPose rz before it reaches the
  // robot: compensates for how the physical toolhead is clocked on the
  // flange relative to the taught/commanded convention. Software-only —
  // never touches the YRC1000's own tool-file/frame definitions. Same value
  // for every tool file: it's one physical toolhead.
  double mount_yaw_offset_deg{0.0};
  std::vector<std::string> deprecations;                 // logged once by the node

  // Throws std::runtime_error with a path-qualified message on any problem.
  static SetupConfig load(const std::string & path);

  const IoAddress * findSensor(const std::string & name) const;
  const IoAddress * findOutput(const std::string & name) const;
  const ToolBinding * findTool(const std::string & name) const;
  const ToolBinding * findToolByNumber(int32_t number) const;  // first match
};

const char * ioKindName(IoKind k);
bool parseIoKind(const std::string & s, IoKind & out);

}  // namespace cobot_control
