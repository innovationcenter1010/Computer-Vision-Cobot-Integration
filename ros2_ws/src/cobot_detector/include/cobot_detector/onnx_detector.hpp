#pragma once
// One loaded ONNX model = one Ort::Session. No ROS.
//
// Load: create the session, read the input geometry [1,3,H,W] and the output
// tensor shape from the graph, classify the head layout (raw / end2end /
// seg — see ModelSpec::layout), and validate it against the head type the
// models.yaml entry claims and against the exporter's manifest.yaml when one
// sits beside the .onnx (task, class count, mask channels). A mislabelled
// model fails at load, not silently at the first scan. Seg models REQUIRE a
// manifest: the class count can't be told apart from the mask-coefficient
// rows by shape alone.
//
// detect(): letterbox → Run → decode (per layout) → NMS (raw/seg only;
// end2end graphs carry their own) → optional duplicate filter → cap at
// max_detections. When the duplicate filter is on, the pre-dedup NMS/decode
// cap is doubled so a duplicate inside that sample can't silently shrink the
// final count below max_detections; the true cap is applied after dedup.
// Thread-safe: Ort::Session::Run is reentrant and all scratch is per-call.

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <onnxruntime_cxx_api.h>

#include "cobot_detector/model_registry.hpp"
#include "cobot_detector/postprocess.hpp"

namespace cobot_detector
{

enum class HeadLayout
{
  kRaw,       // [1, 4+nc(+1), N] — decode + NMS here
  kEnd2End,   // [1, max_det, 6|7] — YOLO26, NMS inside the graph
  kSeg,       // [1, 4+nc+mask_coeffs, N] — YOLOE; boxes only, masks ignored
};

const char * headLayoutName(HeadLayout layout);

class OnnxDetector
{
public:
  struct Result
  {
    std::vector<Detection> detections;
    double inference_ms = 0.0;    // session.Run() only
  };

  // intra_op_threads: 0 = let ORT pick (all cores).
  explicit OnnxDetector(const ModelSpec & spec, int intra_op_threads = 0);

  // confidence_override / max_detections_override: < 0 means "use the
  // model's own models.yaml value" (spec_.confidence / spec_.max_detections).
  // Lets the node apply a live-tunable ROS param on top of the per-model
  // default without editing models.yaml.
  Result detect(
    const cv::Mat & rgb, float confidence_override = -1.0f,
    int max_detections_override = -1) const;

  const ModelSpec & spec() const {return spec_;}
  int inputWidth() const {return in_w_;}
  int inputHeight() const {return in_h_;}
  int numClasses() const {return num_classes_;}
  HeadLayout layout() const {return layout_;}
  const char * layoutName() const {return headLayoutName(layout_);}
  const std::vector<std::string> & classNames() const {return class_names_;}
  const std::string & providers() const {return providers_;}

private:
  static Ort::Env & env();

  ModelSpec spec_;
  Ort::SessionOptions options_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_, output_name_;
  int in_w_ = 0, in_h_ = 0;
  HeadLayout layout_ = HeadLayout::kRaw;
  int out_dim1_ = 0, out_dim2_ = 0;   // graph output is [1, dim1, dim2]
  int num_classes_ = 0;
  std::vector<std::string> class_names_;   // from manifest.yaml if present
  std::string providers_;
};

}  // namespace cobot_detector
