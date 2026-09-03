#pragma once
// Read-once YRC1000 controller profile: system info + tool files (TOOL.CND)
// + user frames (UFRAME.CND). Pure C++ — no ROS, no YMConnect — so the
// parsers run in gtests on fixture text and the node only supplies the
// text-source callbacks (SDK on hardware, files in fixture mode).
//
// The controller is READ-ONLY here: the builder consumes text that the SDK
// already saved from the controller; nothing in this unit can write back.
//
// The exact TOOL.CND / UFRAME.CND layout is confirmed against real dumps
// (test/data). The parsers are deliberately tolerant: anything they do not
// recognise lands in `extra` / `warnings`, never in an exception.

#include <chrono>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace cobot_control
{

struct ToolFileData
{
  int number{-1};
  std::string name;
  double x{0}, y{0}, z{0}, rx{0}, ry{0}, rz{0};  // TCP, mm / deg
  double weight_kg{0};                            // W
  std::vector<double> cog_mm;                     // XG YG ZG (3 or empty)
  std::vector<double> inertia;                    // IX IY IZ (3 or empty)
  std::vector<std::pair<std::string, std::string>> extra;

  bool isIdentity() const;
};

struct UserFrameData
{
  int number{-1};
  std::string name;
  int tool_number{-1};
  int group{-1};
  std::vector<double> pose;  // BUSER x y z rx ry rz (6 or empty)
  std::vector<std::pair<std::string, std::string>> extra;

  bool defined() const;
};

template<class T>
struct ParseResult
{
  std::vector<T> items;
  std::vector<std::string> warnings;
};

ParseResult<ToolFileData> parseToolCnd(const std::string & text);
ParseResult<UserFrameData> parseUframeCnd(const std::string & text);

struct ControllerProfileData
{
  bool loaded{false};
  std::string source{"none"};  // "controller" | "fixture" | "none"
  std::string model_name;
  std::string software_version;
  std::vector<ToolFileData> tools;
  std::vector<UserFrameData> user_frames;
  std::vector<std::string> cnd_files;
  std::vector<std::string> errors;    // per-section failures, non-fatal
  std::vector<std::string> warnings;  // parser warnings
  std::chrono::system_clock::time_point read_at{};

  const ToolFileData * findTool(int number) const;
};

// Text-source abstraction. Every callback returns true on success and fills
// `err` on failure. system_info / list_cnd may be null (section skipped with
// an error entry); read_file is required.
struct ProfileSource
{
  std::function<bool(std::string & model, std::string & version, std::string & err)> system_info;
  std::function<bool(std::vector<std::string> & names, std::string & err)> list_cnd;
  std::function<bool(const std::string & file, std::string & text, std::string & err)> read_file;
};

struct ProfileFiles
{
  std::string tool_file{"TOOL.CND"};
  std::string uframe_file{"UFRAME.CND"};
};

// Never throws. Each section failure becomes an errors[] entry; `loaded` is
// true when at least one section succeeded. raw_out (optional) receives the
// raw file texts keyed by file name so the node can dump them to disk.
ControllerProfileData buildProfile(
  const ProfileSource & source, const ProfileFiles & files,
  std::map<std::string, std::string> * raw_out = nullptr);

// Fixture source: reads <dir>/<file> for read_file, <dir>/system_info.yaml
// (keys model_name / software_version, plain "key: value" lines) for
// system_info, and lists the *.CND files present in <dir> for list_cnd.
ProfileSource fixtureSource(const std::string & dir);

// ISO-8601 UTC, seconds precision.
std::string isoTimestamp(std::chrono::system_clock::time_point tp);

}  // namespace cobot_control
