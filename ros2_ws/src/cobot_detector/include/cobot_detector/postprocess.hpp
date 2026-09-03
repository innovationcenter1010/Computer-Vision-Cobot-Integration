#pragma once
// Everything ultralytics' `predict()` did after the network: decode the raw
// output tensor, threshold, rotated NMS, cap, and the centre-distance
// duplicate filter that yolo_obb_detector.py used to apply. No ROS, no ORT.

#include <array>
#include <vector>

#include <opencv2/core.hpp>

#include "cobot_detector/letterbox.hpp"

namespace cobot_detector
{

struct Detection
{
  int class_id = 0;
  float confidence = 0.0f;
  float cx = 0.0f, cy = 0.0f;   // original-frame pixels
  float angle = 0.0f;           // radians, ultralytics convention; 0 for bbox
  // TL, TR, BR, BL — the ultralytics xyxyxyxy order localization_node relies on
  std::array<cv::Point2f, 4> corners{};
};

struct DecodeParams
{
  float confidence = 0.5f;
  float iou = 0.4f;
  int max_detections = 20;
};

// Raw YOLO output is [1, C, N], row-major: value(c, i) = data[c * anchors + i].
//   OBB : C = 4 + num_classes + 1   (cx, cy, w, h, scores..., angle)
//   bbox: C = 4 + num_classes       (cx, cy, w, h, scores...)
// Returns candidates above `confidence`, already mapped to original-frame
// pixels. Does NOT run NMS.
std::vector<Detection> decodeObb(
  const float * data, int channels, int anchors, const Letterbox & lb, float confidence);
std::vector<Detection> decodeBbox(
  const float * data, int channels, int anchors, const Letterbox & lb, float confidence);

// Segmentation head used as a plain box detector (e.g. YOLOE): same [1, C, N]
// raw layout but C = 4 + num_classes + mask_coeff_channels — the class-score
// block ends where the (ignored) mask coefficients start, so num_classes must
// be given (from the exporter's manifest). Does NOT run NMS.
std::vector<Detection> decodeSegBoxes(
  const float * data, int channels, int anchors, int num_classes,
  const Letterbox & lb, float confidence);

// End-to-end head (YOLO26): output is [1, max_det, cols], one row per final
// detection, NMS already inside the graph.
//   cols = 6: cx, cy, w, h, conf, cls           (detect)
//   cols = 7: cx, cy, w, h, conf, cls, angle    (obb, angle radians)
// (verified empirically on the exported cans_obb_best.onnx). Rows below
// `confidence` are dropped; order (descending conf) is preserved.
std::vector<Detection> decodeEnd2End(
  const float * data, int rows, int cols, const Letterbox & lb, float confidence);

// (cx, cy, w, h, angle) → 4 corners exactly like ultralytics xywhr2xyxyxyxy.
std::array<cv::Point2f, 4> obbCorners(float cx, float cy, float w, float h, float angle);

// Polygon IoU of two (convex) corner quads.
float polygonIou(const std::array<cv::Point2f, 4> & a, const std::array<cv::Point2f, 4> & b);

// Per-class greedy NMS on polygon IoU (works for rotated and axis-aligned),
// highest confidence first, then cap at max_detections.
std::vector<Detection> nms(std::vector<Detection> dets, float iou, int max_detections);

// Port of remove_duplicate_detections(): keep the highest-confidence box,
// drop any later box whose centre is closer than min_center_distance_px.
std::vector<Detection> removeDuplicates(std::vector<Detection> dets, float min_center_distance_px);

}  // namespace cobot_detector
