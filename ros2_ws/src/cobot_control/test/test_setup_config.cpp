#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "cobot_control/setup_config.hpp"

using namespace cobot_control;
namespace fs = std::filesystem;

namespace
{
std::string writeTemp(const std::string & name, const std::string & text)
{
  const fs::path d = fs::temp_directory_path() / "cobot_control_setup_config_test";
  fs::create_directories(d);
  const fs::path p = d / name;
  std::ofstream f(p, std::ios::trunc);
  f << text;
  return p.string();
}
}  // namespace

TEST(SetupConfig, LoadsFixtureToolsAndDefault)
{
  const SetupConfig cfg = SetupConfig::load(std::string(TEST_DATA_DIR) + "/setup_tools.yaml");
  ASSERT_EQ(cfg.tools.size(), 2u);
  ASSERT_NE(cfg.findTool("none"), nullptr);
  EXPECT_EQ(cfg.findTool("none")->tool_number, 0);
  ASSERT_NE(cfg.findTool("vacuum_head"), nullptr);
  EXPECT_EQ(cfg.findTool("vacuum_head")->tool_number, 1);
  EXPECT_EQ(cfg.findToolByNumber(1)->name, "vacuum_head");
  EXPECT_EQ(cfg.findToolByNumber(9), nullptr);
  EXPECT_EQ(cfg.active_tool_default, "none");
  // The deprecated motion_profiles.*.tool_number is accepted and reported.
  ASSERT_EQ(cfg.deprecations.size(), 1u);
  EXPECT_NE(cfg.deprecations[0].find("tool_number"), std::string::npos);
  EXPECT_EQ(cfg.motion_profiles.at("default").accuracy_mm, 50);
}

TEST(SetupConfig, MountYawOffsetDefaultsToZeroAndParses)
{
  EXPECT_EQ(SetupConfig::load(std::string(TEST_DATA_DIR) + "/setup_tools.yaml").mount_yaw_offset_deg, 0.0);
  const std::string p = writeTemp(
    "yaw_offset.yaml", "tools:\n  none: {tool_number: 0}\nmount_yaw_offset_deg: 90.0\n");
  EXPECT_EQ(SetupConfig::load(p).mount_yaw_offset_deg, 90.0);
}

TEST(SetupConfig, NoToolsSectionStillLoads)
{
  const std::string p = writeTemp("no_tools.yaml", "motion_profiles:\n  default: {speed: 10.0}\n");
  const SetupConfig cfg = SetupConfig::load(p);
  EXPECT_TRUE(cfg.tools.empty());
  EXPECT_EQ(cfg.active_tool_default, "none");
}

TEST(SetupConfig, ActiveToolSpellings)
{
  EXPECT_EQ(SetupConfig::load(writeTemp("a.yaml", "tools:\n  none: {tool_number: 0}\nactive_tool: \"\"\n")).active_tool_default, "none");
  EXPECT_EQ(SetupConfig::load(writeTemp("b.yaml", "tools:\n  none: {tool_number: 0}\nactive_tool: none\n")).active_tool_default, "none");
  EXPECT_EQ(SetupConfig::load(writeTemp("c.yaml", "tools:\n  none: {tool_number: 0}\n  g: {tool_number: 3}\nactive_tool: g\n")).active_tool_default, "g");
  EXPECT_THROW(SetupConfig::load(writeTemp("d.yaml", "tools:\n  none: {tool_number: 0}\nactive_tool: ghost\n")), std::runtime_error);
}

TEST(SetupConfig, ToolValidation)
{
  EXPECT_THROW(SetupConfig::load(writeTemp("e.yaml", "tools:\n  g: {tool_number: 64}\n")), std::runtime_error);
  EXPECT_THROW(SetupConfig::load(writeTemp("f.yaml", "tools:\n  g: {tool_number: -1}\n")), std::runtime_error);
  EXPECT_THROW(SetupConfig::load(writeTemp("g.yaml", "tools:\n  none: {tool_number: 2}\n")), std::runtime_error);
  EXPECT_THROW(SetupConfig::load(writeTemp("h.yaml", "tools:\n  g: 5\n")), std::runtime_error);
}
