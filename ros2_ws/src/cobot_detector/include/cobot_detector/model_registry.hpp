#pragma once
// models.yaml → ModelSpec table. No ROS, no ONNX; just yaml-cpp.
//
//   runtime:                  # models the application runs every cycle
//     cans:
//       path: /ws/runtime_models/cans_obb_best/cans_obb_best.onnx
//       type: obb             # obb | bbox
//       confidence: 0.50
//       iou: 0.40
//       max_detections: 20
//       remove_duplicates: true
//       duplicate_center_distance_px: 50.0
//   calibration:              # models only the calibration routine uses
//     red_dot:
//       path: ...
//       type: bbox
//
// The leaf key is the model id used by the Detect action. Ids must be
// unique across both sections. Input geometry is NOT declared here — it is
// read from the ONNX file itself (the exporter fixes it).

#include <map>
#include <string>
#include <vector>

namespace cobot_detector
{

struct ModelSpec
{
  std::string id;
  std::string section;       // "runtime" | "calibration"
  std::string path;          // .onnx, absolute or relative to the yaml's directory
  bool obb = true;           // type: obb → true, bbox → false
  // Head layout: "auto" (default; classified from the graph shape + the
  // exporter's manifest.yaml), or an explicit "raw" | "end2end" | "seg".
  //   raw     — [1, 4+nc(+1), N] head; decode + NMS in C++
  //   end2end — [1, max_det, 6|7] (YOLO26); NMS already inside the graph
  //   seg     — [1, 4+nc+mask_coeffs, N] (YOLOE); boxes only, masks ignored
  std::string layout = "auto";
  float confidence = 0.5f;
  float iou = 0.4f;
  int max_detections = 20;
  bool remove_duplicates = false;
  float duplicate_center_distance_px = 50.0f;
};

class ModelRegistry
{
public:
  static ModelRegistry load(const std::string & yaml_path);   // throws std::runtime_error
  static ModelRegistry parse(const std::string & yaml_text, const std::string & base_dir = "");

  const ModelSpec * find(const std::string & id) const;
  const std::map<std::string, ModelSpec> & all() const {return specs_;}
  std::vector<std::string> ids() const;

private:
  std::map<std::string, ModelSpec> specs_;
};

}  // namespace cobot_detector
