#pragma once
// Ultralytics-style letterbox: scale the image to fit (keeping aspect), pad
// the remainder with grey 114, and write it as float NCHW /255 for the
// network. Keeps enough state to map network-space pixels back to the
// original frame.

#include <vector>

#include <opencv2/core.hpp>

namespace cobot_detector
{

struct Letterbox
{
  int src_w = 0, src_h = 0;   // original frame
  int dst_w = 0, dst_h = 0;   // network input
  float scale = 1.0f;         // src * scale = resized
  int pad_left = 0, pad_top = 0;

  cv::Point2f unmap(const cv::Point2f & p) const
  {
    return {(p.x - static_cast<float>(pad_left)) / scale,
      (p.y - static_cast<float>(pad_top)) / scale};
  }
};

// rgb: 8UC3 HxW. Fills `nchw` (resized to 3*dst_h*dst_w) with planar float
// RGB in [0,1] and returns the mapping.
Letterbox letterbox(const cv::Mat & rgb, int dst_w, int dst_h, std::vector<float> & nchw);

}  // namespace cobot_detector
