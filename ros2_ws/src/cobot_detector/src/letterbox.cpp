#include "cobot_detector/letterbox.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace cobot_detector
{

Letterbox letterbox(const cv::Mat & rgb, int dst_w, int dst_h, std::vector<float> & nchw)
{
  if (rgb.empty() || rgb.type() != CV_8UC3) {
    throw std::invalid_argument("letterbox: expected a non-empty 8UC3 RGB image");
  }
  Letterbox lb;
  lb.src_w = rgb.cols;
  lb.src_h = rgb.rows;
  lb.dst_w = dst_w;
  lb.dst_h = dst_h;

  // Same arithmetic as ultralytics LetterBox(auto=False, scaleup=True):
  //   r = min(H/h, W/w); new = round(w*r), round(h*r); pad split evenly,
  //   top/left = round(pad - 0.1) so a half pixel rounds down.
  lb.scale = std::min(
    static_cast<float>(dst_h) / static_cast<float>(rgb.rows),
    static_cast<float>(dst_w) / static_cast<float>(rgb.cols));
  const int new_w = static_cast<int>(std::lround(rgb.cols * lb.scale));
  const int new_h = static_cast<int>(std::lround(rgb.rows * lb.scale));
  const float dw = (dst_w - new_w) / 2.0f;
  const float dh = (dst_h - new_h) / 2.0f;
  lb.pad_left = static_cast<int>(std::lround(dw - 0.1f));
  lb.pad_top = static_cast<int>(std::lround(dh - 0.1f));

  cv::Mat resized;
  if (new_w == rgb.cols && new_h == rgb.rows) {
    resized = rgb;
  } else {
    cv::resize(rgb, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
  }

  cv::Mat canvas(dst_h, dst_w, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(canvas(cv::Rect(lb.pad_left, lb.pad_top, new_w, new_h)));

  // HWC uint8 → CHW float /255
  const size_t plane = static_cast<size_t>(dst_w) * static_cast<size_t>(dst_h);
  nchw.resize(3 * plane);
  float * r = nchw.data();
  float * g = r + plane;
  float * b = g + plane;
  for (int y = 0; y < dst_h; ++y) {
    const uint8_t * row = canvas.ptr<uint8_t>(y);
    const size_t off = static_cast<size_t>(y) * static_cast<size_t>(dst_w);
    for (int x = 0; x < dst_w; ++x) {
      r[off + x] = row[3 * x + 0] * (1.0f / 255.0f);
      g[off + x] = row[3 * x + 1] * (1.0f / 255.0f);
      b[off + x] = row[3 * x + 2] * (1.0f / 255.0f);
    }
  }
  return lb;
}

}  // namespace cobot_detector
