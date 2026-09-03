#pragma once
// Persisted active-tool selection (<runtime_dir>/tooling_state.yaml).
// Written by cobot_control on /cobot/set_active_tool, read at boot so a
// restart (or a launch respawn after a crash) never silently flips the
// commanded TCP back to tool file 0. Plain yaml-cpp, no ROS.

#include <cstdint>
#include <string>

namespace cobot_control
{

struct ActiveTool
{
  int32_t number{0};
  std::string name{"none"};
  std::string source{"setup_default"};  // setup_default | state_file | service
  bool verified{false};                 // number found in the parsed TOOL.CND
};

struct ToolingState
{
  // false (with err) when the file is missing or unreadable; `out` untouched.
  static bool load(const std::string & path, ActiveTool & out, std::string & err);
  // Creates the parent directory, writes <path>.tmp, renames over <path>.
  static bool save(const std::string & path, const ActiveTool & tool, std::string & err);
};

}  // namespace cobot_control
