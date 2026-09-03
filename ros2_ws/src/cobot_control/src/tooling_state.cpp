#include "cobot_control/tooling_state.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>

namespace cobot_control
{

namespace fs = std::filesystem;

bool ToolingState::load(const std::string & path, ActiveTool & out, std::string & err)
{
  if (path.empty()) {err = "no path"; return false;}
  if (!fs::exists(path)) {err = path + " does not exist"; return false;}
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    err = std::string("cannot read ") + path + ": " + e.what();
    return false;
  }
  const YAML::Node at = root["active_tool"];
  if (!at || !at.IsMap()) {err = path + ": missing `active_tool` map"; return false;}
  try {
    ActiveTool t;
    t.name = at["name"] ? at["name"].as<std::string>() : "";
    t.number = at["tool_number"] ? at["tool_number"].as<int>() : -1;
    if (t.name.empty()) {err = path + ": active_tool.name is empty"; return false;}
    if (t.number < 0 || t.number > 63) {
      err = path + ": active_tool.tool_number out of range 0-63";
      return false;
    }
    t.source = "state_file";
    out = t;
    return true;
  } catch (const std::exception & e) {
    err = std::string(path) + ": " + e.what();
    return false;
  }
}

bool ToolingState::save(const std::string & path, const ActiveTool & tool, std::string & err)
{
  if (path.empty()) {err = "no path (runtime_dir unset)"; return false;}
  try {
    const fs::path p(path);
    if (p.has_parent_path()) {fs::create_directories(p.parent_path());}
    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&now));

    YAML::Emitter out;
    out << YAML::Comment(
      "written by cobot_control (/cobot/set_active_tool); do not hand-edit while running");
    out << YAML::BeginMap << YAML::Key << "active_tool" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << tool.name;
    out << YAML::Key << "tool_number" << YAML::Value << tool.number;
    out << YAML::Key << "set_at" << YAML::Value << std::string(stamp);
    out << YAML::Key << "set_by" << YAML::Value << "/cobot/set_active_tool";
    out << YAML::EndMap << YAML::EndMap;

    const fs::path tmp = p.string() + ".tmp";
    {
      std::ofstream f(tmp, std::ios::trunc);
      if (!f) {err = "cannot open " + tmp.string(); return false;}
      f << out.c_str() << "\n";
    }
    fs::rename(tmp, p);
    return true;
  } catch (const std::exception & e) {
    err = std::string("cannot write ") + path + ": " + e.what();
    return false;
  }
}

}  // namespace cobot_control
