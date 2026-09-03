#include "cobot_detector/postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace cobot_detector
{

namespace
{

// Best class for anchor i over rows [4, 4 + nc).
inline void bestClass(
  const float * data, int anchors, int nc, int i, int & cls, float & score)
{
  cls = 0;
  score = data[4 * anchors + i];
  for (int c = 1; c < nc; ++c) {
    const float s = data[(4 + c) * anchors + i];
    if (s > score) {
      score = s;
      cls = c;
    }
  }
}

}  // namespace

std::array<cv::Point2f, 4> obbCorners(float cx, float cy, float w, float h, float angle)
{
  // ultralytics.utils.ops.xywhr2xyxyxyxy:
  //   vec1 = [w/2*cos, w/2*sin], vec2 = [-h/2*sin, h/2*cos]
  //   pt1 = ctr + vec1 + vec2, pt2 = ctr + vec1 - vec2,
  //   pt3 = ctr - vec1 - vec2, pt4 = ctr - vec1 + vec2
  const float c = std::cos(angle), s = std::sin(angle);
  const cv::Point2f v1(w / 2.0f * c, w / 2.0f * s);
  const cv::Point2f v2(-h / 2.0f * s, h / 2.0f * c);
  const cv::Point2f ctr(cx, cy);
  return {ctr + v1 + v2, ctr + v1 - v2, ctr - v1 - v2, ctr - v1 + v2};
}

std::vector<Detection> decodeObb(
  const float * data, int channels, int anchors, const Letterbox & lb, float confidence)
{
  const int nc = channels - 5;
  if (nc < 1) {
    throw std::invalid_argument("decodeObb: output has " + std::to_string(channels) +
            " channels, need >= 6 (cx,cy,w,h,score...,angle)");
  }
  std::vector<Detection> out;
  for (int i = 0; i < anchors; ++i) {
    int cls;
    float score;
    bestClass(data, anchors, nc, i, cls, score);
    if (score < confidence) {continue;}

    const float cx = data[0 * anchors + i];
    const float cy = data[1 * anchors + i];
    const float w = data[2 * anchors + i];
    const float h = data[3 * anchors + i];
    const float angle = data[(channels - 1) * anchors + i];

    Detection d;
    d.class_id = cls;
    d.confidence = score;
    d.angle = angle;
    const auto net = obbCorners(cx, cy, w, h, angle);
    for (int k = 0; k < 4; ++k) {d.corners[k] = lb.unmap(net[k]);}
    // Centre as the corner mean — identical to what the Python node published.
    d.cx = (d.corners[0].x + d.corners[1].x + d.corners[2].x + d.corners[3].x) / 4.0f;
    d.cy = (d.corners[0].y + d.corners[1].y + d.corners[2].y + d.corners[3].y) / 4.0f;
    out.push_back(d);
  }
  return out;
}

std::vector<Detection> decodeBbox(
  const float * data, int channels, int anchors, const Letterbox & lb, float confidence)
{
  const int nc = channels - 4;
  if (nc < 1) {
    throw std::invalid_argument("decodeBbox: output has " + std::to_string(channels) +
            " channels, need >= 5 (cx,cy,w,h,score...)");
  }
  std::vector<Detection> out;
  for (int i = 0; i < anchors; ++i) {
    int cls;
    float score;
    bestClass(data, anchors, nc, i, cls, score);
    if (score < confidence) {continue;}

    const float cx = data[0 * anchors + i];
    const float cy = data[1 * anchors + i];
    const float w = data[2 * anchors + i];
    const float h = data[3 * anchors + i];

    const cv::Point2f tl = lb.unmap({cx - w / 2.0f, cy - h / 2.0f});
    const cv::Point2f br = lb.unmap({cx + w / 2.0f, cy + h / 2.0f});

    Detection d;
    d.class_id = cls;
    d.confidence = score;
    d.angle = 0.0f;
    d.corners = {tl, cv::Point2f(br.x, tl.y), br, cv::Point2f(tl.x, br.y)};
    d.cx = (tl.x + br.x) / 2.0f;
    d.cy = (tl.y + br.y) / 2.0f;
    out.push_back(d);
  }
  return out;
}

std::vector<Detection> decodeSegBoxes(
  const float * data, int channels, int anchors, int num_classes,
  const Letterbox & lb, float confidence)
{
  if (num_classes < 1 || channels < 4 + num_classes) {
    throw std::invalid_argument(
            "decodeSegBoxes: " + std::to_string(channels) + " channels can't hold 4 box rows + " +
            std::to_string(num_classes) + " class scores");
  }
  std::vector<Detection> out;
  for (int i = 0; i < anchors; ++i) {
    int cls;
    float score;
    bestClass(data, anchors, num_classes, i, cls, score);
    if (score < confidence) {continue;}

    const float cx = data[0 * anchors + i];
    const float cy = data[1 * anchors + i];
    const float w = data[2 * anchors + i];
    const float h = data[3 * anchors + i];

    const cv::Point2f tl = lb.unmap({cx - w / 2.0f, cy - h / 2.0f});
    const cv::Point2f br = lb.unmap({cx + w / 2.0f, cy + h / 2.0f});

    Detection d;
    d.class_id = cls;
    d.confidence = score;
    d.angle = 0.0f;
    d.corners = {tl, cv::Point2f(br.x, tl.y), br, cv::Point2f(tl.x, br.y)};
    d.cx = (tl.x + br.x) / 2.0f;
    d.cy = (tl.y + br.y) / 2.0f;
    out.push_back(d);
  }
  return out;
}

std::vector<Detection> decodeEnd2End(
  const float * data, int rows, int cols, const Letterbox & lb, float confidence)
{
  if (cols != 6 && cols != 7) {
    throw std::invalid_argument(
            "decodeEnd2End: expected 6 (detect) or 7 (obb) columns, got " + std::to_string(cols));
  }
  std::vector<Detection> out;
  for (int i = 0; i < rows; ++i) {
    const float * r = data + static_cast<size_t>(i) * cols;
    const float conf = r[4];
    if (conf < confidence) {continue;}

    Detection d;
    d.confidence = conf;
    d.class_id = static_cast<int>(r[5]);
    d.angle = (cols == 7) ? r[6] : 0.0f;
    const auto net = obbCorners(r[0], r[1], r[2], r[3], d.angle);
    for (int k = 0; k < 4; ++k) {d.corners[k] = lb.unmap(net[k]);}
    d.cx = (d.corners[0].x + d.corners[1].x + d.corners[2].x + d.corners[3].x) / 4.0f;
    d.cy = (d.corners[0].y + d.corners[1].y + d.corners[2].y + d.corners[3].y) / 4.0f;
    out.push_back(d);
  }
  return out;
}

float polygonIou(const std::array<cv::Point2f, 4> & a, const std::array<cv::Point2f, 4> & b)
{
  const std::vector<cv::Point2f> pa(a.begin(), a.end());
  const std::vector<cv::Point2f> pb(b.begin(), b.end());
  const double area_a = std::abs(cv::contourArea(pa));
  const double area_b = std::abs(cv::contourArea(pb));
  if (area_a <= 0.0 || area_b <= 0.0) {return 0.0f;}

  std::vector<cv::Point2f> inter;
  const double area_i = std::abs(cv::intersectConvexConvex(pa, pb, inter, true));
  const double uni = area_a + area_b - area_i;
  return uni <= 0.0 ? 0.0f : static_cast<float>(area_i / uni);
}

std::vector<Detection> nms(std::vector<Detection> dets, float iou, int max_detections)
{
  std::stable_sort(
    dets.begin(), dets.end(),
    [](const Detection & x, const Detection & y) {return x.confidence > y.confidence;});

  std::vector<Detection> kept;
  std::vector<bool> suppressed(dets.size(), false);
  for (size_t i = 0; i < dets.size(); ++i) {
    if (suppressed[i]) {continue;}
    kept.push_back(dets[i]);
    if (static_cast<int>(kept.size()) >= max_detections) {break;}
    for (size_t j = i + 1; j < dets.size(); ++j) {
      if (suppressed[j] || dets[j].class_id != dets[i].class_id) {continue;}
      if (polygonIou(dets[i].corners, dets[j].corners) > iou) {suppressed[j] = true;}
    }
  }
  return kept;
}

std::vector<Detection> removeDuplicates(std::vector<Detection> dets, float min_center_distance_px)
{
  std::stable_sort(
    dets.begin(), dets.end(),
    [](const Detection & x, const Detection & y) {return x.confidence > y.confidence;});

  std::vector<Detection> kept;
  for (const auto & d : dets) {
    bool duplicate = false;
    for (const auto & k : kept) {
      const float dx = d.cx - k.cx, dy = d.cy - k.cy;
      if (std::sqrt(dx * dx + dy * dy) < min_center_distance_px) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {kept.push_back(d);}
  }
  return kept;
}

}  // namespace cobot_detector
