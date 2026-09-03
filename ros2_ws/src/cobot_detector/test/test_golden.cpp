// Golden test: run the real exported models on real frames and compare with
// what the retired ultralytics node produced for the same frames.
//
// Files (produced by vision_workspace/tools/dump_golden.py, gitignored
// except for the small sample checked in):
//   test/data/golden/<frame>.png
//   test/data/golden/<frame>.<model_id>.txt     one detection per line:
//       class conf cx cy x1 y1 x2 y2 x3 y3 x4 y4
//
// Skips (does not fail) when the models or the golden files are absent so
// the package still builds and tests on a machine without the mount.

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "cobot_detector/model_registry.hpp"
#include "cobot_detector/onnx_detector.hpp"

namespace fs = std::filesystem;
using namespace cobot_detector;

namespace
{

struct GoldenDet
{
  int cls;
  float conf, cx, cy;
  std::array<cv::Point2f, 4> corners;
};

std::vector<GoldenDet> readGolden(const fs::path & p)
{
  std::vector<GoldenDet> out;
  std::ifstream in(p);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream ss(line);
    GoldenDet g;
    ss >> g.cls >> g.conf >> g.cx >> g.cy;
    for (auto & c : g.corners) {ss >> c.x >> c.y;}
    out.push_back(g);
  }
  return out;
}

void compareModel(const ModelRegistry & reg, const std::string & id, const fs::path & dir)
{
  const auto * spec = reg.find(id);
  ASSERT_NE(spec, nullptr);
  if (!fs::exists(spec->path)) {
    GTEST_SKIP() << "model file absent: " << spec->path;
  }
  std::vector<fs::path> goldens;
  for (const auto & e : fs::directory_iterator(dir)) {
    const auto name = e.path().filename().string();
    if (name.size() > id.size() + 5 && name.substr(name.size() - id.size() - 4) == id + ".txt") {
      goldens.push_back(e.path());
    }
  }
  if (goldens.empty()) {
    GTEST_SKIP() << "no golden files for '" << id << "' in " << dir;
  }

  OnnxDetector det(*spec);
  for (const auto & g : goldens) {
    const std::string stem = g.filename().string();
    const fs::path png = dir / (stem.substr(0, stem.size() - id.size() - 5) + ".png");
    cv::Mat bgr = cv::imread(png.string(), cv::IMREAD_COLOR);
    ASSERT_FALSE(bgr.empty()) << png;
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);

    const auto expected = readGolden(g);
    const auto got = det.detect(rgb).detections;
    EXPECT_EQ(got.size(), expected.size()) << png;

    // Match each expected detection to the nearest produced centre.
    for (const auto & e : expected) {
      float best = 1e9f;
      const Detection * match = nullptr;
      for (const auto & d : got) {
        const float dist = std::hypot(d.cx - e.cx, d.cy - e.cy);
        if (dist < best) {best = dist; match = &d;}
      }
      ASSERT_NE(match, nullptr) << png;
      EXPECT_LT(best, 2.0f) << png << " centre off by " << best << " px";
      EXPECT_EQ(match->class_id, e.cls) << png;
      EXPECT_NEAR(match->confidence, e.conf, 0.05f) << png;
      for (int k = 0; k < 4; ++k) {
        EXPECT_LT(std::hypot(match->corners[k].x - e.corners[k].x,
          match->corners[k].y - e.corners[k].y), 3.0f) << png << " corner " << k;
      }
    }
  }
}

}  // namespace

TEST(Golden, Cans)
{
  const fs::path dir = fs::path(TEST_DATA_DIR) / "golden";
  if (!fs::exists(dir)) {GTEST_SKIP() << "no golden dir " << dir;}
  compareModel(ModelRegistry::load((fs::path(TEST_DATA_DIR) / "models.yaml").string()), "cans", dir);
}

TEST(Golden, RedDot)
{
  const fs::path dir = fs::path(TEST_DATA_DIR) / "golden";
  if (!fs::exists(dir)) {GTEST_SKIP() << "no golden dir " << dir;}
  compareModel(ModelRegistry::load((fs::path(TEST_DATA_DIR) / "models.yaml").string()), "red_dot", dir);
}
