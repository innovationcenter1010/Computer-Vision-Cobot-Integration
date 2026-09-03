#include "cobot_control/controller_profile.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace cobot_control
{

namespace fs = std::filesystem;

// ------------------------------------------------------------------ helpers

namespace
{

std::string trim(const std::string & s)
{
  size_t b = 0, e = s.size();
  while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) {++b;}
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) {--e;}
  return s.substr(b, e - b);
}

std::string upper(std::string s)
{
  for (auto & c : s) {c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));}
  return s;
}

std::vector<std::string> splitLines(const std::string & text)
{
  std::vector<std::string> lines;
  std::string cur;
  for (char c : text) {
    if (c == '\n') {lines.push_back(cur); cur.clear();} else if (c != '\r') {cur += c;}
  }
  if (!cur.empty()) {lines.push_back(cur);}
  return lines;
}

// Numbers separated by commas and/or whitespace. Returns false when any
// token is not a number (so "TOOL1" is never mistaken for data).
bool parseNumbers(const std::string & s, std::vector<double> & out)
{
  out.clear();
  std::string t = s;
  for (auto & c : t) {if (c == ',' || c == '\t') {c = ' ';}}
  std::istringstream in(t);
  std::string tok;
  while (in >> tok) {
    char * end = nullptr;
    const double v = std::strtod(tok.c_str(), &end);
    if (end == tok.c_str() || *end != '\0') {return false;}
    out.push_back(v);
  }
  return !out.empty();
}

bool parseInt(const std::string & s, int & out)
{
  std::vector<double> v;
  if (!parseNumbers(s, v) || v.size() != 1) {return false;}
  out = static_cast<int>(std::lround(v[0]));
  return true;
}

// Block header: "//TOOL 3" / "//UFRAME 12" (also tolerates "//NAME 3", the
// older spelling some dumps use). Returns the block number or -1.
int blockNumber(const std::string & line, const std::string & block_key)
{
  if (line.size() < 3 || line.compare(0, 2, "//") != 0 || line[2] == '/') {return -1;}
  const std::string body = trim(line.substr(2));
  const auto sp = body.find_first_of(" \t");
  if (sp == std::string::npos) {return -1;}
  const std::string key = upper(body.substr(0, sp));
  if (key != block_key && key != "NAME") {return -1;}
  int n = -1;
  return parseInt(trim(body.substr(sp)), n) ? n : -1;
}

// Keyed line: "///X 12.5" -> ("X", "12.5"). Returns false otherwise.
bool keyedLine(const std::string & line, std::string & key, std::string & value)
{
  if (line.compare(0, 3, "///") != 0) {return false;}
  const std::string body = trim(line.substr(3));
  if (body.empty()) {return false;}
  const auto sp = body.find_first_of(" \t");
  key = upper(sp == std::string::npos ? body : body.substr(0, sp));
  value = sp == std::string::npos ? "" : trim(body.substr(sp));
  return !key.empty();
}

template<class T, class OnHeader, class OnKeyed, class OnBare>
ParseResult<T> parseBlocks(
  const std::string & text, const std::string & block_key, const std::string & file_tag,
  OnHeader on_header, OnKeyed on_keyed, OnBare on_bare)
{
  ParseResult<T> result;
  std::vector<T> items;
  T * cur = nullptr;
  int stray = 0;
  for (const std::string & raw : splitLines(text)) {
    const std::string line = trim(raw);
    if (line.empty()) {continue;}
    // File header "/TOOL" / "/UFRAME" (single slash).
    if (line[0] == '/' && (line.size() == 1 || line[1] != '/')) {continue;}
    const int n = blockNumber(line, block_key);
    if (n >= 0) {
      auto dup = std::find_if(items.begin(), items.end(), [&](const T & t) {return t.number == n;});
      if (dup != items.end()) {
        result.warnings.push_back(file_tag + ": duplicate block " + std::to_string(n) + " (last wins)");
        *dup = T{};
        dup->number = n;
        cur = &(*dup);
      } else {
        items.push_back(T{});
        cur = &items.back();
        cur->number = n;
      }
      on_header(*cur);
      continue;
    }
    if (!cur) {++stray; continue;}
    std::string key, value;
    if (keyedLine(line, key, value)) {
      on_keyed(*cur, key, value, result.warnings);
    } else {
      on_bare(*cur, line, result.warnings);
    }
  }
  if (stray > 0) {
    result.warnings.push_back(
      file_tag + ": " + std::to_string(stray) + " line(s) before the first block ignored");
  }
  if (items.empty()) {result.warnings.push_back(file_tag + ": no blocks found");}
  std::sort(items.begin(), items.end(), [](const T & a, const T & b) {return a.number < b.number;});
  result.items = std::move(items);
  return result;
}

}  // namespace

// ------------------------------------------------------------------ data

bool ToolFileData::isIdentity() const
{
  const double eps = 1e-6;
  return std::fabs(x) < eps && std::fabs(y) < eps && std::fabs(z) < eps &&
         std::fabs(rx) < eps && std::fabs(ry) < eps && std::fabs(rz) < eps;
}

bool UserFrameData::defined() const
{
  if (!pose.empty() || !name.empty()) {return true;}
  for (const auto & kv : extra) {
    if (kv.first == "RORG" || kv.first == "RXX" || kv.first == "RXY") {return true;}
  }
  return false;
}

const ToolFileData * ControllerProfileData::findTool(int number) const
{
  for (const auto & t : tools) {if (t.number == number) {return &t;}}
  return nullptr;
}

// ------------------------------------------------------------------ TOOL.CND

ParseResult<ToolFileData> parseToolCnd(const std::string & text)
{
  return parseBlocks<ToolFileData>(
    text, "TOOL", "TOOL.CND",
    [](ToolFileData &) {},
    [](ToolFileData & t, const std::string & key, const std::string & value,
    std::vector<std::string> & warnings) {
      auto num = [&](double & slot) {
          std::vector<double> v;
          if (parseNumbers(value, v) && v.size() == 1) {slot = v[0]; return true;}
          warnings.push_back(
            "TOOL.CND tool " + std::to_string(t.number) + ": " + key + " '" + value +
            "' is not one number");
          t.extra.emplace_back(key, value);
          return false;
        };
      auto triple = [&](std::vector<double> & slot, size_t index) {
          std::vector<double> v;
          if (parseNumbers(value, v) && v.size() == 1) {
            if (slot.size() != 3) {slot.assign(3, 0.0);}
            slot[index] = v[0];
            return;
          }
          warnings.push_back(
            "TOOL.CND tool " + std::to_string(t.number) + ": " + key + " '" + value +
            "' is not one number");
          t.extra.emplace_back(key, value);
        };
      if (key == "NAME") {t.name = value; return;}
      if (key == "X") {num(t.x); return;}
      if (key == "Y") {num(t.y); return;}
      if (key == "Z") {num(t.z); return;}
      if (key == "RX") {num(t.rx); return;}
      if (key == "RY") {num(t.ry); return;}
      if (key == "RZ") {num(t.rz); return;}
      if (key == "W") {num(t.weight_kg); return;}
      if (key == "XG") {triple(t.cog_mm, 0); return;}
      if (key == "YG") {triple(t.cog_mm, 1); return;}
      if (key == "ZG") {triple(t.cog_mm, 2); return;}
      if (key == "IX") {triple(t.inertia, 0); return;}
      if (key == "IY") {triple(t.inertia, 1); return;}
      if (key == "IZ") {triple(t.inertia, 2); return;}
      // Combined spellings some dumps use.
      std::vector<double> v;
      if ((key == "XYZ" || key == "TCP") && parseNumbers(value, v) && v.size() == 6) {
        t.x = v[0]; t.y = v[1]; t.z = v[2]; t.rx = v[3]; t.ry = v[4]; t.rz = v[5];
        return;
      }
      if (key == "XYZG" && parseNumbers(value, v) && v.size() == 3) {t.cog_mm = v; return;}
      if (key == "IXYZ" && parseNumbers(value, v) && v.size() == 3) {t.inertia = v; return;}
      t.extra.emplace_back(key, value);
    },
    [](ToolFileData & t, const std::string & line, std::vector<std::string> & warnings) {
      // A bare line of 6 numbers is the TCP (x y z rx ry rz).
      std::vector<double> v;
      if (parseNumbers(line, v) && v.size() == 6) {
        t.x = v[0]; t.y = v[1]; t.z = v[2]; t.rx = v[3]; t.ry = v[4]; t.rz = v[5];
        return;
      }
      warnings.push_back(
        "TOOL.CND tool " + std::to_string(t.number) + ": unrecognised line '" + line + "'");
      t.extra.emplace_back("?", line);
    });
}

// ------------------------------------------------------------------ UFRAME.CND

ParseResult<UserFrameData> parseUframeCnd(const std::string & text)
{
  return parseBlocks<UserFrameData>(
    text, "UFRAME", "UFRAME.CND",
    [](UserFrameData &) {},
    [](UserFrameData & f, const std::string & key, const std::string & value,
    std::vector<std::string> & warnings) {
      if (key == "NAME") {f.name = value; return;}
      if (key == "TOOL") {
        int n = -1;
        if (parseInt(value, n)) {f.tool_number = n; return;}
      } else if (key == "GROUP") {
        int n = -1;
        if (parseInt(value, n)) {f.group = n; return;}
        // GROUP is sometimes binary notation ("0001") — keep the raw text.
      } else if (key == "BUSER") {
        std::vector<double> v;
        if (parseNumbers(value, v) && v.size() == 6) {f.pose = v; return;}
        warnings.push_back(
          "UFRAME.CND frame " + std::to_string(f.number) + ": BUSER '" + value +
          "' is not 6 numbers");
      }
      f.extra.emplace_back(key, value);
    },
    [](UserFrameData & f, const std::string & line, std::vector<std::string> & warnings) {
      warnings.push_back(
        "UFRAME.CND frame " + std::to_string(f.number) + ": unrecognised line '" + line + "'");
      f.extra.emplace_back("?", line);
    });
}

// ------------------------------------------------------------------ builder

std::string isoTimestamp(std::chrono::system_clock::time_point tp)
{
  const std::time_t t = std::chrono::system_clock::to_time_t(tp);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&t));
  return buf;
}

ControllerProfileData buildProfile(
  const ProfileSource & source, const ProfileFiles & files,
  std::map<std::string, std::string> * raw_out)
{
  ControllerProfileData p;
  p.read_at = std::chrono::system_clock::now();
  bool any = false;
  std::string err;

  if (source.system_info) {
    if (source.system_info(p.model_name, p.software_version, err)) {any = true;} else {
      p.errors.push_back("system_info: " + err);
    }
  } else {
    p.errors.push_back("system_info: no source");
  }

  if (source.list_cnd) {
    std::vector<std::string> names;
    if (source.list_cnd(names, err)) {p.cnd_files = names; any = true;} else {
      p.errors.push_back("cnd_list: " + err);
    }
  }

  auto readSection = [&](const std::string & file, const std::string & tag, std::string & text) {
      if (!source.read_file) {p.errors.push_back(tag + ": no source"); return false;}
      err.clear();
      if (!source.read_file(file, text, err)) {
        p.errors.push_back(tag + ": " + (err.empty() ? "read failed" : err));
        return false;
      }
      if (raw_out) {(*raw_out)[file] = text;}
      return true;
    };

  std::string text;
  if (readSection(files.tool_file, "tool_files", text)) {
    auto r = parseToolCnd(text);
    p.tools = std::move(r.items);
    p.warnings.insert(p.warnings.end(), r.warnings.begin(), r.warnings.end());
    any = true;
  }
  text.clear();
  if (readSection(files.uframe_file, "user_frames", text)) {
    auto r = parseUframeCnd(text);
    p.user_frames = std::move(r.items);
    p.warnings.insert(p.warnings.end(), r.warnings.begin(), r.warnings.end());
    any = true;
  }

  p.loaded = any;
  return p;
}

ProfileSource fixtureSource(const std::string & dir)
{
  ProfileSource s;
  s.read_file = [dir](const std::string & file, std::string & text, std::string & err) {
      const fs::path p = fs::path(dir) / file;
      std::ifstream f(p);
      if (!f) {err = "fixture " + p.string() + " not found"; return false;}
      std::ostringstream ss;
      ss << f.rdbuf();
      text = ss.str();
      return true;
    };
  s.system_info = [dir](std::string & model, std::string & version, std::string & err) {
      const fs::path p = fs::path(dir) / "system_info.yaml";
      std::ifstream f(p);
      if (!f) {err = "fixture " + p.string() + " not found"; return false;}
      std::string line;
      while (std::getline(f, line)) {
        const auto colon = line.find(':');
        if (colon == std::string::npos) {continue;}
        const std::string key = trim(line.substr(0, colon));
        std::string value = trim(line.substr(colon + 1));
        if (value.size() >= 2 && (value.front() == '"' || value.front() == '\'') &&
          value.back() == value.front())
        {
          value = value.substr(1, value.size() - 2);
        }
        if (key == "model_name") {model = value;} else if (key == "software_version") {
          version = value;
        }
      }
      return true;
    };
  s.list_cnd = [dir](std::vector<std::string> & names, std::string & err) {
      std::error_code ec;
      if (!fs::is_directory(dir, ec)) {err = "fixture dir " + dir + " not found"; return false;}
      for (const auto & entry : fs::directory_iterator(dir, ec)) {
        const std::string n = entry.path().filename().string();
        if (upper(entry.path().extension().string()) == ".CND") {names.push_back(n);}
      }
      std::sort(names.begin(), names.end());
      return true;
    };
  return s;
}

}  // namespace cobot_control
