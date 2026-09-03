// Parser + builder tests on the fixtures in test/data (see README there: the
// fixtures are SYNTHETIC until replaced with real controller dumps).
#include <gtest/gtest.h>

#include <fstream>
#include <sstream>

#include "cobot_control/controller_profile.hpp"

using namespace cobot_control;

namespace
{

std::string readFixture(const std::string & name)
{
  std::ifstream f(std::string(TEST_DATA_DIR) + "/" + name);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

}  // namespace

TEST(ToolCnd, ParsesFixture)
{
  const auto r = parseToolCnd(readFixture("TOOL.CND"));
  ASSERT_EQ(r.items.size(), 3u);
  EXPECT_TRUE(r.warnings.empty()) << r.warnings.front();

  EXPECT_EQ(r.items[0].number, 0);
  EXPECT_TRUE(r.items[0].isIdentity());
  EXPECT_EQ(r.items[0].name, "");

  const ToolFileData & t1 = r.items[1];
  EXPECT_EQ(t1.number, 1);
  EXPECT_EQ(t1.name, "VACUUM_HEAD");
  EXPECT_DOUBLE_EQ(t1.z, 600.0);
  EXPECT_DOUBLE_EQ(t1.weight_kg, 4.5);
  ASSERT_EQ(t1.cog_mm.size(), 3u);
  EXPECT_DOUBLE_EQ(t1.cog_mm[2], 300.0);
  ASSERT_EQ(t1.inertia.size(), 3u);
  EXPECT_DOUBLE_EQ(t1.inertia[2], 0.02);
  EXPECT_FALSE(t1.isIdentity());

  const ToolFileData & t2 = r.items[2];
  EXPECT_DOUBLE_EQ(t2.x, 12.5);
  EXPECT_DOUBLE_EQ(t2.y, -3.0);
  EXPECT_DOUBLE_EQ(t2.rz, 45.0);
}

TEST(ToolCnd, LineEndingsAndWhitespaceDoNotMatter)
{
  const std::string crlf = readFixture("TOOL.CND");
  std::string lf;
  for (char c : crlf) {if (c != '\r') {lf += c;}}
  std::string tabs;
  for (char c : lf) {tabs += (c == ' ') ? '\t' : c;}
  const auto a = parseToolCnd(crlf);
  const auto b = parseToolCnd(lf);
  const auto c = parseToolCnd(tabs);
  ASSERT_EQ(a.items.size(), b.items.size());
  ASSERT_EQ(a.items.size(), c.items.size());
  for (size_t i = 0; i < a.items.size(); ++i) {
    EXPECT_EQ(a.items[i].name, b.items[i].name);
    EXPECT_DOUBLE_EQ(a.items[i].z, c.items[i].z);
    EXPECT_DOUBLE_EQ(a.items[i].weight_kg, c.items[i].weight_kg);
  }
}

TEST(ToolCnd, ToleratesGarbageUnknownKeysAndBadCounts)
{
  const std::string text =
    "garbage before any block\n"
    "//TOOL 5\n"
    "///NAME ODD\n"
    "///X 1.0\n"
    "///FOO bar baz\n"
    "///W 1 2\n"
    "1 2 3 4 5\n"
    "10 20 30 0 0 90\n";
  const auto r = parseToolCnd(text);
  ASSERT_EQ(r.items.size(), 1u);
  const ToolFileData & t = r.items[0];
  EXPECT_EQ(t.number, 5);
  EXPECT_EQ(t.name, "ODD");
  // The bare 6-number line is the TCP and overrides the keyed X.
  EXPECT_DOUBLE_EQ(t.x, 10.0);
  EXPECT_DOUBLE_EQ(t.rz, 90.0);
  EXPECT_EQ(t.weight_kg, 0.0);
  ASSERT_GE(t.extra.size(), 3u);  // FOO, W (bad), the 5-number line
  EXPECT_EQ(t.extra[0].first, "FOO");
  EXPECT_GE(r.warnings.size(), 3u);  // stray line, W count, 5-number line
}

TEST(ToolCnd, DuplicateBlocksLastWinsWithWarning)
{
  const std::string text = "//TOOL 1\n///Z 100\n//TOOL 1\n///Z 200\n";
  const auto r = parseToolCnd(text);
  ASSERT_EQ(r.items.size(), 1u);
  EXPECT_DOUBLE_EQ(r.items[0].z, 200.0);
  ASSERT_EQ(r.warnings.size(), 1u);
  EXPECT_NE(r.warnings[0].find("duplicate"), std::string::npos);
}

TEST(ToolCnd, EmptyText)
{
  const auto r = parseToolCnd("");
  EXPECT_TRUE(r.items.empty());
  ASSERT_EQ(r.warnings.size(), 1u);
}

TEST(ToolCnd, SortedByNumber)
{
  const auto r = parseToolCnd("//TOOL 7\n///Z 7\n//TOOL 2\n///Z 2\n");
  ASSERT_EQ(r.items.size(), 2u);
  EXPECT_EQ(r.items[0].number, 2);
  EXPECT_EQ(r.items[1].number, 7);
}

TEST(UframeCnd, ParsesFixture)
{
  const auto r = parseUframeCnd(readFixture("UFRAME.CND"));
  ASSERT_EQ(r.items.size(), 2u);
  EXPECT_TRUE(r.warnings.empty()) << r.warnings.front();
  const UserFrameData & f = r.items[0];
  EXPECT_EQ(f.number, 1);
  EXPECT_EQ(f.name, "BIN");
  EXPECT_EQ(f.tool_number, 1);
  EXPECT_EQ(f.group, 1);
  ASSERT_EQ(f.pose.size(), 6u);
  EXPECT_DOUBLE_EQ(f.pose[1], -950.0);
  EXPECT_DOUBLE_EQ(f.pose[5], 90.0);
  EXPECT_TRUE(f.defined());
  // Everything else is kept raw.
  bool saw_rorg = false;
  for (const auto & kv : f.extra) {if (kv.first == "RORG") {saw_rorg = true;}}
  EXPECT_TRUE(saw_rorg);

  const UserFrameData & g = r.items[1];
  EXPECT_EQ(g.tool_number, 0);
  EXPECT_TRUE(g.pose.empty());
  EXPECT_FALSE(g.defined());
}

TEST(Builder, PartialFailureIsNonFatal)
{
  ProfileSource src = fixtureSource(TEST_DATA_DIR);
  const auto real_read = src.read_file;
  src.read_file = [real_read](const std::string & file, std::string & text, std::string & err) {
      if (file == "UFRAME.CND") {err = "[1234] simulated refusal"; return false;}
      return real_read(file, text, err);
    };
  std::map<std::string, std::string> raw;
  const auto p = buildProfile(src, ProfileFiles{}, &raw);
  EXPECT_TRUE(p.loaded);
  ASSERT_EQ(p.errors.size(), 1u);
  EXPECT_NE(p.errors[0].find("user_frames"), std::string::npos);
  EXPECT_EQ(p.tools.size(), 3u);
  EXPECT_TRUE(p.user_frames.empty());
  EXPECT_EQ(raw.count("TOOL.CND"), 1u);
  EXPECT_EQ(raw.count("UFRAME.CND"), 0u);
  EXPECT_EQ(p.model_name, "YR-1-06VXH12-A00");
  EXPECT_NE(p.findTool(1), nullptr);
  EXPECT_EQ(p.findTool(9), nullptr);
}

TEST(Builder, EverythingFailing)
{
  ProfileSource src;
  src.system_info = [](std::string &, std::string &, std::string & err) {err = "off"; return false;};
  src.list_cnd = [](std::vector<std::string> &, std::string & err) {err = "off"; return false;};
  src.read_file = [](const std::string &, std::string &, std::string & err) {err = "off"; return false;};
  const auto p = buildProfile(src, ProfileFiles{});
  EXPECT_FALSE(p.loaded);
  EXPECT_EQ(p.errors.size(), 4u);
  EXPECT_TRUE(p.tools.empty());
}

TEST(Builder, FixtureSourceEndToEnd)
{
  const auto p = buildProfile(fixtureSource(TEST_DATA_DIR), ProfileFiles{});
  EXPECT_TRUE(p.loaded);
  EXPECT_TRUE(p.errors.empty()) << p.errors.front();
  EXPECT_EQ(p.tools.size(), 3u);
  EXPECT_EQ(p.user_frames.size(), 2u);
  ASSERT_EQ(p.cnd_files.size(), 2u);
  EXPECT_EQ(p.cnd_files[0], "TOOL.CND");
  EXPECT_EQ(p.software_version, "YAS2.94.00A(JP/US)-00");
  EXPECT_EQ(isoTimestamp(p.read_at).size(), 20u);
}
