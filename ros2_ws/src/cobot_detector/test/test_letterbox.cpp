#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "cobot_detector/letterbox.hpp"

using cobot_detector::Letterbox;
using cobot_detector::letterbox;

TEST(Letterbox, SquareInputNoPadding)
{
  cv::Mat img(640, 640, CV_8UC3, cv::Scalar(10, 20, 30));
  std::vector<float> nchw;
  const Letterbox lb = letterbox(img, 640, 640, nchw);
  EXPECT_FLOAT_EQ(lb.scale, 1.0f);
  EXPECT_EQ(lb.pad_left, 0);
  EXPECT_EQ(lb.pad_top, 0);
  ASSERT_EQ(nchw.size(), 3u * 640 * 640);
  // planar R, G, B
  EXPECT_NEAR(nchw[0], 10 / 255.0f, 1e-6);
  EXPECT_NEAR(nchw[640 * 640], 20 / 255.0f, 1e-6);
  EXPECT_NEAR(nchw[2 * 640 * 640], 30 / 255.0f, 1e-6);
}

TEST(Letterbox, CameraFrameIntoSquareNetwork)
{
  // 640x480 camera frame → 640x640 net: scale 1, 80 px pad top and bottom.
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  std::vector<float> nchw;
  const Letterbox lb = letterbox(img, 640, 640, nchw);
  EXPECT_FLOAT_EQ(lb.scale, 1.0f);
  EXPECT_EQ(lb.pad_left, 0);
  EXPECT_EQ(lb.pad_top, 80);
  // padding is grey 114
  EXPECT_NEAR(nchw[0], 114 / 255.0f, 1e-6);            // R, row 0 (pad)
  EXPECT_NEAR(nchw[80 * 640], 0.0f, 1e-6);             // R, row 80 (image)
  // unmap: net (100, 180) → frame (100, 100)
  const cv::Point2f p = lb.unmap({100.0f, 180.0f});
  EXPECT_NEAR(p.x, 100.0f, 1e-4);
  EXPECT_NEAR(p.y, 100.0f, 1e-4);
}

TEST(Letterbox, RectangularNetworkMatchesFrame)
{
  // The exporter's default is a 480x640 (HxW) input — 1:1 with the camera.
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  std::vector<float> nchw;
  const Letterbox lb = letterbox(img, 640, 480, nchw);
  EXPECT_FLOAT_EQ(lb.scale, 1.0f);
  EXPECT_EQ(lb.pad_left, 0);
  EXPECT_EQ(lb.pad_top, 0);
  ASSERT_EQ(nchw.size(), 3u * 640 * 480);
}

TEST(Letterbox, DownscaleRoundTrip)
{
  cv::Mat img(720, 1280, CV_8UC3, cv::Scalar(0, 0, 0));
  std::vector<float> nchw;
  const Letterbox lb = letterbox(img, 640, 640, nchw);
  EXPECT_FLOAT_EQ(lb.scale, 0.5f);
  EXPECT_EQ(lb.pad_left, 0);
  EXPECT_EQ(lb.pad_top, 140);
  const cv::Point2f p = lb.unmap({320.0f, 320.0f});
  EXPECT_NEAR(p.x, 640.0f, 1e-3);
  EXPECT_NEAR(p.y, 360.0f, 1e-3);
}

TEST(Letterbox, RejectsWrongType)
{
  cv::Mat gray(480, 640, CV_8UC1);
  std::vector<float> nchw;
  EXPECT_THROW(letterbox(gray, 640, 640, nchw), std::invalid_argument);
}
