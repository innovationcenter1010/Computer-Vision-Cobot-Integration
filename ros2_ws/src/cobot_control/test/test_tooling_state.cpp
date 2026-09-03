#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "cobot_control/tooling_state.hpp"

using namespace cobot_control;
namespace fs = std::filesystem;

namespace
{
fs::path tmpDir()
{
  const fs::path d = fs::temp_directory_path() / "cobot_control_tooling_state_test";
  fs::remove_all(d);
  return d;  // save() must create it
}
}  // namespace

TEST(ToolingState, RoundTripCreatesParentDir)
{
  const fs::path path = tmpDir() / "nested" / "tooling_state.yaml";
  ActiveTool t;
  t.number = 2;
  t.name = "gripper";
  t.source = "service";
  std::string err;
  ASSERT_TRUE(ToolingState::save(path.string(), t, err)) << err;
  EXPECT_FALSE(fs::exists(path.string() + ".tmp"));

  ActiveTool back;
  ASSERT_TRUE(ToolingState::load(path.string(), back, err)) << err;
  EXPECT_EQ(back.number, 2);
  EXPECT_EQ(back.name, "gripper");
  EXPECT_EQ(back.source, "state_file");
}

TEST(ToolingState, MissingFile)
{
  ActiveTool t;
  std::string err;
  EXPECT_FALSE(ToolingState::load((tmpDir() / "nope.yaml").string(), t, err));
  EXPECT_FALSE(err.empty());
  EXPECT_EQ(t.number, 0);  // untouched
}

TEST(ToolingState, MalformedFile)
{
  const fs::path d = tmpDir();
  fs::create_directories(d);
  const fs::path p = d / "bad.yaml";
  {std::ofstream f(p); f << "active_tool: [not, a, map]\n";}
  ActiveTool t;
  std::string err;
  EXPECT_FALSE(ToolingState::load(p.string(), t, err));
  {std::ofstream f(p); f << "active_tool: {name: x, tool_number: 99}\n";}
  EXPECT_FALSE(ToolingState::load(p.string(), t, err));
  EXPECT_NE(err.find("0-63"), std::string::npos);
}

TEST(ToolingState, EmptyPathRefused)
{
  ActiveTool t;
  std::string err;
  EXPECT_FALSE(ToolingState::save("", t, err));
  EXPECT_FALSE(ToolingState::load("", t, err));
}
