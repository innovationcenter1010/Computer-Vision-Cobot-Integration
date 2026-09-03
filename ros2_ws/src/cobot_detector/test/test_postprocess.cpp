#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "cobot_detector/postprocess.hpp"

using namespace cobot_detector;

namespace
{

Letterbox identity(int w = 640, int h = 640)
{
  Letterbox lb;
  lb.src_w = lb.dst_w = w;
  lb.src_h = lb.dst_h = h;
  lb.scale = 1.0f;
  return lb;
}

// Build a [1, C, N] tensor (row-major, channel-major) from per-anchor rows.
std::vector<float> tensor(const std::vector<std::vector<float>> & anchors_rows)
{
  const int n = static_cast<int>(anchors_rows.size());
  const int c = static_cast<int>(anchors_rows[0].size());
  std::vector<float> t(static_cast<size_t>(c) * n, 0.0f);
  for (int i = 0; i < n; ++i) {
    for (int k = 0; k < c; ++k) {t[k * n + i] = anchors_rows[i][k];}
  }
  return t;
}

}  // namespace

TEST(ObbCorners, AxisAlignedMatchesUltralyticsOrder)
{
  // angle 0: pt1 = ctr + (w/2, 0) + (0, h/2) = bottom-right... in ultralytics
  // convention pt1..pt4 = (+v1+v2, +v1-v2, -v1-v2, -v1+v2).
  const auto c = obbCorners(100, 100, 40, 20, 0.0f);
  EXPECT_NEAR(c[0].x, 120, 1e-4); EXPECT_NEAR(c[0].y, 110, 1e-4);
  EXPECT_NEAR(c[1].x, 120, 1e-4); EXPECT_NEAR(c[1].y, 90, 1e-4);
  EXPECT_NEAR(c[2].x, 80, 1e-4);  EXPECT_NEAR(c[2].y, 90, 1e-4);
  EXPECT_NEAR(c[3].x, 80, 1e-4);  EXPECT_NEAR(c[3].y, 110, 1e-4);
}

TEST(ObbCorners, RotatedBoxKeepsSideLengths)
{
  const float w = 60, h = 30, a = 0.7f;
  const auto c = obbCorners(200, 150, w, h, a);
  const auto len = [](cv::Point2f p, cv::Point2f q) {return std::hypot(p.x - q.x, p.y - q.y);};
  EXPECT_NEAR(len(c[0], c[1]), h, 1e-3);
  EXPECT_NEAR(len(c[1], c[2]), w, 1e-3);
  EXPECT_NEAR(len(c[2], c[3]), h, 1e-3);
  EXPECT_NEAR(len(c[3], c[0]), w, 1e-3);
}

TEST(DecodeObb, ThresholdsAndBestClass)
{
  // C = 4 + 2 classes + 1 angle = 7
  const auto t = tensor({
    {100, 100, 40, 20, 0.9f, 0.1f, 0.0f},    // class 0, keep
    {300, 300, 40, 20, 0.2f, 0.6f, 0.5f},    // class 1, keep
    {500, 500, 40, 20, 0.3f, 0.2f, 0.0f},    // below threshold
  });
  const auto d = decodeObb(t.data(), 7, 3, identity(), 0.5f);
  ASSERT_EQ(d.size(), 2u);
  EXPECT_EQ(d[0].class_id, 0);
  EXPECT_FLOAT_EQ(d[0].confidence, 0.9f);
  EXPECT_NEAR(d[0].cx, 100, 1e-3);
  EXPECT_NEAR(d[0].cy, 100, 1e-3);
  EXPECT_EQ(d[1].class_id, 1);
  EXPECT_FLOAT_EQ(d[1].angle, 0.5f);
}

TEST(DecodeObb, UnmapsThroughLetterbox)
{
  Letterbox lb;
  lb.src_w = 640; lb.src_h = 480; lb.dst_w = lb.dst_h = 640;
  lb.scale = 1.0f; lb.pad_left = 0; lb.pad_top = 80;
  const auto t = tensor({{320, 320, 40, 20, 0.9f, 0.0f}});   // C = 6, nc = 1
  const auto d = decodeObb(t.data(), 6, 1, lb, 0.5f);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_NEAR(d[0].cx, 320, 1e-3);
  EXPECT_NEAR(d[0].cy, 240, 1e-3);
}

TEST(DecodeBbox, AxisAlignedCornersAngleZero)
{
  // C = 4 + 1 class = 5
  const auto t = tensor({{100, 100, 40, 20, 0.8f}});
  const auto d = decodeBbox(t.data(), 5, 1, identity(), 0.5f);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_FLOAT_EQ(d[0].angle, 0.0f);
  EXPECT_NEAR(d[0].corners[0].x, 80, 1e-4);  EXPECT_NEAR(d[0].corners[0].y, 90, 1e-4);   // TL
  EXPECT_NEAR(d[0].corners[1].x, 120, 1e-4); EXPECT_NEAR(d[0].corners[1].y, 90, 1e-4);   // TR
  EXPECT_NEAR(d[0].corners[2].x, 120, 1e-4); EXPECT_NEAR(d[0].corners[2].y, 110, 1e-4);  // BR
  EXPECT_NEAR(d[0].corners[3].x, 80, 1e-4);  EXPECT_NEAR(d[0].corners[3].y, 110, 1e-4);  // BL
  EXPECT_NEAR(d[0].cx, 100, 1e-4);
  EXPECT_NEAR(d[0].cy, 100, 1e-4);
}

TEST(Decode, RejectsTooFewChannels)
{
  std::vector<float> t(5, 0.0f);
  EXPECT_THROW(decodeObb(t.data(), 5, 1, identity(), 0.5f), std::invalid_argument);
  EXPECT_THROW(decodeBbox(t.data(), 4, 1, identity(), 0.5f), std::invalid_argument);
}

TEST(PolygonIou, IdenticalAndDisjoint)
{
  const auto a = obbCorners(100, 100, 40, 20, 0.3f);
  const auto b = obbCorners(400, 400, 40, 20, 0.3f);
  EXPECT_NEAR(polygonIou(a, a), 1.0f, 1e-4);
  EXPECT_NEAR(polygonIou(a, b), 0.0f, 1e-6);
}

TEST(PolygonIou, HalfOverlap)
{
  const auto a = obbCorners(100, 100, 40, 20, 0.0f);
  const auto b = obbCorners(120, 100, 40, 20, 0.0f);   // shifted by w/2
  EXPECT_NEAR(polygonIou(a, b), 1.0f / 3.0f, 1e-4);      // 400 / (800+800-400)
}

TEST(Nms, SuppressesOverlapSameClassOnly)
{
  Detection strong; strong.class_id = 0; strong.confidence = 0.9f;
  strong.corners = obbCorners(100, 100, 40, 20, 0.0f); strong.cx = 100; strong.cy = 100;
  Detection weak = strong; weak.confidence = 0.6f;
  weak.corners = obbCorners(104, 100, 40, 20, 0.0f); weak.cx = 104;
  Detection other = weak; other.class_id = 1;

  const auto kept = nms({weak, strong, other}, 0.4f, 20);
  ASSERT_EQ(kept.size(), 2u);
  EXPECT_FLOAT_EQ(kept[0].confidence, 0.9f);   // sorted, strong first
  EXPECT_EQ(kept[1].class_id, 1);              // different class survives
}

TEST(Nms, CapsAtMaxDetections)
{
  std::vector<Detection> dets;
  for (int i = 0; i < 10; ++i) {
    Detection d; d.confidence = 0.5f + i * 0.01f;
    d.corners = obbCorners(50.0f + 100.0f * i, 100, 40, 20, 0.0f); d.cx = 50.0f + 100.0f * i;
    dets.push_back(d);
  }
  EXPECT_EQ(nms(dets, 0.4f, 3).size(), 3u);
}

TEST(DecodeSegBoxes, IgnoresMaskCoefficientRows)
{
  // C = 4 box + 2 classes + 3 mask coeffs = 9; coeffs hold huge values that
  // must NOT be read as class scores.
  const auto t = tensor({
    {100, 100, 40, 20, 0.9f, 0.1f, 99.0f, -99.0f, 50.0f},
    {300, 300, 40, 20, 0.1f, 0.2f, 99.0f, 99.0f, 99.0f},   // below threshold
  });
  const auto d = decodeSegBoxes(t.data(), 9, 2, 2, identity(), 0.5f);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_EQ(d[0].class_id, 0);
  EXPECT_FLOAT_EQ(d[0].confidence, 0.9f);
  EXPECT_FLOAT_EQ(d[0].angle, 0.0f);
  EXPECT_NEAR(d[0].corners[0].x, 80, 1e-4);
  EXPECT_NEAR(d[0].corners[2].y, 110, 1e-4);
}

TEST(DecodeSegBoxes, RejectsImpossibleClassCount)
{
  std::vector<float> t(9, 0.0f);
  EXPECT_THROW(decodeSegBoxes(t.data(), 9, 1, 6, identity(), 0.5f), std::invalid_argument);
}

TEST(DecodeEnd2End, ObbRows)
{
  // [rows=3, cols=7]: cx cy w h conf cls angle — row-major per detection.
  const std::vector<float> t = {
    320, 240, 100, 50, 0.99f, 0, 0.3f,
    100, 100, 40, 20, 0.80f, 1, -0.1f,
    500, 400, 40, 20, 0.10f, 0, 0.0f,      // below threshold
  };
  const auto d = decodeEnd2End(t.data(), 3, 7, identity(), 0.5f);
  ASSERT_EQ(d.size(), 2u);
  EXPECT_NEAR(d[0].cx, 320, 1e-3);
  EXPECT_NEAR(d[0].cy, 240, 1e-3);
  EXPECT_FLOAT_EQ(d[0].angle, 0.3f);
  EXPECT_EQ(d[1].class_id, 1);
  // corner side lengths match w/h
  const auto len = [](cv::Point2f p, cv::Point2f q) {return std::hypot(p.x - q.x, p.y - q.y);};
  EXPECT_NEAR(len(d[0].corners[1], d[0].corners[2]), 100, 1e-3);
  EXPECT_NEAR(len(d[0].corners[0], d[0].corners[1]), 50, 1e-3);
}

TEST(DecodeEnd2End, DetectRowsUnmapThroughLetterbox)
{
  Letterbox lb;
  lb.src_w = 640; lb.src_h = 480; lb.dst_w = lb.dst_h = 640;
  lb.scale = 1.0f; lb.pad_left = 0; lb.pad_top = 80;
  const std::vector<float> t = {320, 320, 40, 20, 0.9f, 2};
  const auto d = decodeEnd2End(t.data(), 1, 6, lb, 0.5f);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_NEAR(d[0].cx, 320, 1e-3);
  EXPECT_NEAR(d[0].cy, 240, 1e-3);   // pad removed
  EXPECT_EQ(d[0].class_id, 2);
  EXPECT_FLOAT_EQ(d[0].angle, 0.0f);
}

TEST(DecodeEnd2End, RejectsBadColumnCount)
{
  std::vector<float> t(8, 0.0f);
  EXPECT_THROW(decodeEnd2End(t.data(), 1, 8, identity(), 0.5f), std::invalid_argument);
}

TEST(RemoveDuplicates, PortOfPythonFilter)
{
  Detection a; a.confidence = 0.9f; a.cx = 100; a.cy = 100;
  Detection b; b.confidence = 0.7f; b.cx = 130; b.cy = 100;   // 30 px away → dup at 50
  Detection c; c.confidence = 0.8f; c.cx = 200; c.cy = 100;   // far → kept
  const auto kept = removeDuplicates({b, c, a}, 50.0f);
  ASSERT_EQ(kept.size(), 2u);
  EXPECT_FLOAT_EQ(kept[0].confidence, 0.9f);
  EXPECT_FLOAT_EQ(kept[1].confidence, 0.8f);
}
