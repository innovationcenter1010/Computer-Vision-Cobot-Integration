#include "cobot_control/setup_config.hpp"

#include <yaml-cpp/yaml.h>

namespace cobot_control
{

const char * ioKindName(IoKind k)
{
  return k == IoKind::GeneralInput ? "general_input" : "general_output";
}

bool parseIoKind(const std::string & s, IoKind & out)
{
  if (s == "general_input" || s == "input") {out = IoKind::GeneralInput; return true;}
  if (s == "general_output" || s == "output") {out = IoKind::GeneralOutput; return true;}
  return false;
}

namespace
{

std::vector<IoAddress> parseIoMap(const YAML::Node & node, const std::string & where)
{
  std::vector<IoAddress> out;
  if (!node) {return out;}
  if (!node.IsMap()) {
    throw std::runtime_error(where + " must be a map of name -> {io_type, group, bit}");
  }
  for (const auto & kv : node) {
    IoAddress a;
    a.name = kv.first.as<std::string>();
    const YAML::Node & v = kv.second;
    const std::string here = where + "." + a.name;
    if (!v.IsMap()) {throw std::runtime_error(here + " must be {io_type, group, bit}");}
    const std::string kind = v["io_type"] ? v["io_type"].as<std::string>() : "general_input";
    if (!parseIoKind(kind, a.kind)) {
      throw std::runtime_error(here + ".io_type must be general_input or general_output");
    }
    if (!v["group"] || !v["bit"]) {throw std::runtime_error(here + " needs group and bit");}
    const int group = v["group"].as<int>();
    const int bit = v["bit"].as<int>();
    if (group < 0 || group > 65535) {throw std::runtime_error(here + ".group out of range");}
    if (bit < 0 || bit > 7) {throw std::runtime_error(here + ".bit must be 0-7");}
    a.group = static_cast<uint16_t>(group);
    a.bit = static_cast<uint8_t>(bit);
    out.push_back(a);
  }
  return out;
}

MotionProfile parseProfile(
  const YAML::Node & v, const std::string & where, MotionProfile base,
  std::vector<std::string> & deprecations)
{
  if (!v.IsMap()) {throw std::runtime_error(where + " must be a map");}
  if (v["interpolation"]) {
    const std::string s = v["interpolation"].as<std::string>();
    if (s == "joint") {
      base.interpolation = Interp::Joint;
    } else if (s == "linear") {
      base.interpolation = Interp::Linear;
    } else {
      throw std::runtime_error(where + ".interpolation must be joint or linear");
    }
  }
  if (v["speed"]) {base.speed = v["speed"].as<double>();}
  if (v["accel_ratio"]) {base.accel_ratio = v["accel_ratio"].as<double>();}
  if (v["decel_ratio"]) {base.decel_ratio = v["decel_ratio"].as<double>();}
  if (v["accuracy_mm"]) {base.accuracy_mm = v["accuracy_mm"].as<int>();}
  if (v["tool_number"]) {
    // Motion profiles describe HOW to move; the tool file (TCP) is the
    // node-level active tool now (/cobot/set_active_tool). Accepted, ignored.
    deprecations.push_back(
      where + ".tool_number is ignored: the tool file comes from the active tool "
      "(cobot_setup.yaml tools / /cobot/set_active_tool)");
  }
  if (base.speed <= 0.0) {throw std::runtime_error(where + ".speed must be > 0");}
  if (base.accel_ratio < 20.0 || base.accel_ratio > 100.0 ||
    base.decel_ratio < 20.0 || base.decel_ratio > 100.0)
  {
    throw std::runtime_error(where + ": accel/decel ratio must be 20-100 (YMConnect)");
  }
  return base;
}

}  // namespace

SetupConfig SetupConfig::load(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    throw std::runtime_error("cannot read setup file '" + path + "': " + e.what());
  }
  SetupConfig cfg;
  try {
    if (root["io"]) {
      cfg.sensors = parseIoMap(root["io"]["sensors"], "io.sensors");
      cfg.outputs = parseIoMap(root["io"]["outputs"], "io.outputs");
    }
    if (root["motion_profiles"]) {
      const YAML::Node profiles = root["motion_profiles"];
      if (!profiles.IsMap()) {throw std::runtime_error("motion_profiles must be a map");}
      for (const auto & kv : profiles) {
        const std::string name = kv.first.as<std::string>();
        cfg.motion_profiles[name] =
          parseProfile(kv.second, "motion_profiles." + name, MotionProfile{}, cfg.deprecations);
      }
    }
    if (root["tools"]) {
      const YAML::Node tools = root["tools"];
      if (!tools.IsMap()) {throw std::runtime_error("tools must be a map of name -> {tool_number, ...}");}
      for (const auto & kv : tools) {
        ToolBinding b;
        b.name = kv.first.as<std::string>();
        const std::string here = "tools." + b.name;
        if (!kv.second.IsMap()) {throw std::runtime_error(here + " must be a map");}
        const int n = kv.second["tool_number"] ? kv.second["tool_number"].as<int>() : 0;
        if (n < 0 || n > 63) {throw std::runtime_error(here + ".tool_number must be 0-63");}
        if (b.name == "none" && n != 0) {
          throw std::runtime_error(here + ".tool_number must be 0 ('none' is the tool-0 default)");
        }
        b.tool_number = static_cast<int32_t>(n);
        cfg.tools.push_back(b);
      }
    }
    if (root["mount_yaw_offset_deg"]) {
      cfg.mount_yaw_offset_deg = root["mount_yaw_offset_deg"].as<double>();
    }
    if (root["active_tool"]) {
      const std::string a = root["active_tool"].IsNull() ? "" : root["active_tool"].as<std::string>();
      if (!a.empty() && a != "none") {
        if (!cfg.findTool(a)) {
          throw std::runtime_error("active_tool '" + a + "' is not a key of tools");
        }
        cfg.active_tool_default = a;
      }
    }
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(path + ": " + e.what());
  }
  if (!cfg.motion_profiles.count("default")) {cfg.motion_profiles["default"] = MotionProfile{};}
  return cfg;
}

const IoAddress * SetupConfig::findSensor(const std::string & name) const
{
  for (const auto & a : sensors) {if (a.name == name) {return &a;}}
  return nullptr;
}

const IoAddress * SetupConfig::findOutput(const std::string & name) const
{
  for (const auto & a : outputs) {if (a.name == name) {return &a;}}
  return nullptr;
}

const ToolBinding * SetupConfig::findTool(const std::string & name) const
{
  for (const auto & t : tools) {if (t.name == name) {return &t;}}
  return nullptr;
}

const ToolBinding * SetupConfig::findToolByNumber(int32_t number) const
{
  for (const auto & t : tools) {if (t.tool_number == number) {return &t;}}
  return nullptr;
}

}  // namespace cobot_control
