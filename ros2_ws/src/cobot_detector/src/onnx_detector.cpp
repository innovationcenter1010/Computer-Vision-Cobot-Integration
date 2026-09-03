#include "cobot_detector/onnx_detector.hpp"

#include <chrono>
#include <filesystem>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "cobot_detector/letterbox.hpp"

namespace cobot_detector
{

namespace
{

std::string shapeText(const std::vector<int64_t> & s)
{
  std::string t = "[";
  for (size_t i = 0; i < s.size(); ++i) {
    t += (i ? "," : "") + std::to_string(s[i]);
  }
  return t + "]";
}

}  // namespace

const char * headLayoutName(HeadLayout layout)
{
  switch (layout) {
    case HeadLayout::kRaw: return "raw";
    case HeadLayout::kEnd2End: return "end2end";
    case HeadLayout::kSeg: return "seg";
  }
  return "?";
}

Ort::Env & OnnxDetector::env()
{
  // One process-wide environment (logging + thread pools) shared by every
  // session, as the ORT docs recommend.
  static Ort::Env instance(ORT_LOGGING_LEVEL_WARNING, "cobot_detector");
  return instance;
}

OnnxDetector::OnnxDetector(const ModelSpec & spec, int intra_op_threads)
: spec_(spec)
{
  if (!std::filesystem::exists(spec_.path)) {
    throw std::runtime_error("model '" + spec_.id + "': file not found: " + spec_.path);
  }

  options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  if (intra_op_threads > 0) {options_.SetIntraOpNumThreads(intra_op_threads);}
  providers_ = "CPUExecutionProvider";

  try {
    session_ = std::make_unique<Ort::Session>(env(), spec_.path.c_str(), options_);
  } catch (const Ort::Exception & e) {
    throw std::runtime_error("model '" + spec_.id + "': ONNX Runtime failed to load " +
            spec_.path + ": " + e.what());
  }

  Ort::AllocatorWithDefaultOptions alloc;
  if (session_->GetInputCount() != 1) {
    throw std::runtime_error("model '" + spec_.id + "': expected exactly one input, got " +
            std::to_string(session_->GetInputCount()));
  }
  input_name_ = session_->GetInputNameAllocated(0, alloc).get();
  output_name_ = session_->GetOutputNameAllocated(0, alloc).get();

  const auto in_shape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  if (in_shape.size() != 4 || in_shape[1] != 3 || in_shape[2] <= 0 || in_shape[3] <= 0) {
    throw std::runtime_error("model '" + spec_.id + "': input shape " + shapeText(in_shape) +
            " is not a static [1,3,H,W] (export with dynamic=False)");
  }
  in_h_ = static_cast<int>(in_shape[2]);
  in_w_ = static_cast<int>(in_shape[3]);

  const auto out_shape = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  if (out_shape.size() != 3 || out_shape[0] != 1 || out_shape[1] <= 0 || out_shape[2] <= 0) {
    throw std::runtime_error("model '" + spec_.id + "': output shape " + shapeText(out_shape) +
            " is not a static [1,A,B] tensor");
  }
  out_dim1_ = static_cast<int>(out_shape[1]);
  out_dim2_ = static_cast<int>(out_shape[2]);

  // ---- exporter manifest (beside the .onnx), if present ----
  std::string manifest_task;        // obb | detect | segment | ""
  std::string manifest_layout;      // e.g. yolo_obb | yolo_detect | yolo_seg | ""
  int manifest_mask_coeffs = -1;
  const auto manifest = std::filesystem::path(spec_.path).parent_path() / "manifest.yaml";
  try {
    if (std::filesystem::exists(manifest)) {
      const YAML::Node m = YAML::LoadFile(manifest.string());
      if (m["task"]) {manifest_task = m["task"].as<std::string>();}
      if (m["output"] && m["output"]["layout"]) {
        manifest_layout = m["output"]["layout"].as<std::string>();
      }
      if (m["output"] && m["output"]["mask_coeff_channels"]) {
        manifest_mask_coeffs = m["output"]["mask_coeff_channels"].as<int>();
      }
      if (m["classes"] && m["classes"].IsSequence()) {
        for (const auto & c : m["classes"]) {class_names_.push_back(c.as<std::string>());}
      }
    }

    if (!manifest_task.empty() && manifest_task != "obb" && manifest_task != "detect" &&
      manifest_task != "segment")
    {
      throw std::runtime_error("manifest task '" + manifest_task + "' is not a detector");
    }
    if (manifest_task == "obb" && !spec_.obb) {
      throw std::runtime_error("models.yaml says type bbox but manifest.yaml says task obb");
    }
    if ((manifest_task == "detect" || manifest_task == "segment") && spec_.obb) {
      throw std::runtime_error(
              "models.yaml says type obb but manifest.yaml says task " + manifest_task);
    }

    // ---- classify the head layout ----
    const bool shape_end2end = (out_dim2_ == 6 || out_dim2_ == 7) && out_dim1_ > out_dim2_;
    const bool manifest_seg = manifest_layout == "yolo_seg" || manifest_mask_coeffs > 0 ||
      manifest_task == "segment";
    if (spec_.layout == "end2end") {
      layout_ = HeadLayout::kEnd2End;
    } else if (spec_.layout == "seg") {
      layout_ = HeadLayout::kSeg;
    } else if (spec_.layout == "raw") {
      layout_ = HeadLayout::kRaw;
    } else {  // auto
      layout_ = shape_end2end ? HeadLayout::kEnd2End :
        (manifest_seg ? HeadLayout::kSeg : HeadLayout::kRaw);
    }

    // ---- per-layout validation; num_classes_ is what decode will use ----
    switch (layout_) {
      case HeadLayout::kEnd2End: {
          const int want_cols = spec_.obb ? 7 : 6;
          if (out_dim2_ != want_cols) {
            throw std::runtime_error(
                    "end2end output " + shapeText(out_shape) + " has " +
                    std::to_string(out_dim2_) + " columns, but type " +
                    (spec_.obb ? "obb" : "bbox") + " needs " + std::to_string(want_cols) +
                    " (cx,cy,w,h,conf,cls" + (spec_.obb ? ",angle)" : ")"));
          }
          num_classes_ = class_names_.empty() ? 1 : static_cast<int>(class_names_.size());
          break;
        }
      case HeadLayout::kSeg: {
          if (spec_.obb) {
            throw std::runtime_error("seg layout produces axis-aligned boxes — use type: bbox");
          }
          if (class_names_.empty()) {
            throw std::runtime_error(
                    "seg layout needs the exporter's manifest.yaml (classes list) beside the "
                    ".onnx — the class count can't be inferred from the shape " +
                    shapeText(out_shape));
          }
          num_classes_ = static_cast<int>(class_names_.size());
          const int mask_coeffs = out_dim1_ - 4 - num_classes_;
          if (mask_coeffs < 1) {
            throw std::runtime_error(
                    "seg output " + shapeText(out_shape) + " can't hold 4 box rows + " +
                    std::to_string(num_classes_) + " classes + mask coefficients");
          }
          if (manifest_mask_coeffs > 0 && manifest_mask_coeffs != mask_coeffs) {
            throw std::runtime_error(
                    "manifest says " + std::to_string(manifest_mask_coeffs) +
                    " mask coefficients but the graph has " + std::to_string(mask_coeffs));
          }
          break;
        }
      case HeadLayout::kRaw: {
          num_classes_ = out_dim1_ - (spec_.obb ? 5 : 4);
          if (num_classes_ < 1) {
            throw std::runtime_error("output has " + std::to_string(out_dim1_) +
                    " channels, too few for a raw " + (spec_.obb ? "obb" : "bbox") + " head");
          }
          if (!class_names_.empty() &&
            static_cast<int>(class_names_.size()) != num_classes_)
          {
            throw std::runtime_error(
                    "manifest lists " + std::to_string(class_names_.size()) +
                    " classes but the raw graph head has " + std::to_string(num_classes_) +
                    " — wrong type/layout in models.yaml?");
          }
          break;
        }
    }
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("model '" + spec_.id + "': bad manifest.yaml: " + e.what());
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("model '" + spec_.id + "': " + e.what());
  }
}

OnnxDetector::Result OnnxDetector::detect(
  const cv::Mat & rgb, float confidence_override, int max_detections_override) const
{
  const float confidence = confidence_override >= 0.0f ? confidence_override : spec_.confidence;
  const int max_detections =
    max_detections_override >= 1 ? max_detections_override : spec_.max_detections;

  std::vector<float> input;
  const Letterbox lb = letterbox(rgb, in_w_, in_h_, input);

  const std::array<int64_t, 4> in_dims{1, 3, in_h_, in_w_};
  Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value in_tensor = Ort::Value::CreateTensor<float>(
    mem, input.data(), input.size(), in_dims.data(), in_dims.size());

  const char * in_names[] = {input_name_.c_str()};
  const char * out_names[] = {output_name_.c_str()};

  const auto t0 = std::chrono::steady_clock::now();
  std::vector<Ort::Value> outputs;
  try {
    outputs = session_->Run(Ort::RunOptions{nullptr}, in_names, &in_tensor, 1, out_names, 1);
  } catch (const Ort::Exception & e) {
    throw std::runtime_error("model '" + spec_.id + "': inference failed: " + e.what());
  }
  const auto t1 = std::chrono::steady_clock::now();

  const auto shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
  if (shape.size() != 3 || shape[1] != out_dim1_ || shape[2] != out_dim2_) {
    throw std::runtime_error("model '" + spec_.id + "': unexpected output shape " +
            shapeText(shape));
  }
  const float * data = outputs[0].GetTensorData<float>();

  // When the duplicate filter runs, cap the pre-dedup sample at double
  // max_detections instead of max_detections itself — otherwise a duplicate
  // landing inside the cap silently shrinks the final count below what the
  // param asked for, even though a further unique detection existed just
  // past the cutoff. Trim to the real max_detections after dedup runs.
  const int sample_cap = spec_.remove_duplicates ? max_detections * 2 : max_detections;

  std::vector<Detection> dets;
  switch (layout_) {
    case HeadLayout::kRaw:
      dets = spec_.obb ?
        decodeObb(data, out_dim1_, out_dim2_, lb, confidence) :
        decodeBbox(data, out_dim1_, out_dim2_, lb, confidence);
      dets = nms(std::move(dets), spec_.iou, sample_cap);
      break;
    case HeadLayout::kSeg:
      dets = decodeSegBoxes(data, out_dim1_, out_dim2_, num_classes_, lb, confidence);
      dets = nms(std::move(dets), spec_.iou, sample_cap);
      break;
    case HeadLayout::kEnd2End:
      // NMS already ran inside the graph; rows come sorted by confidence.
      dets = decodeEnd2End(data, out_dim1_, out_dim2_, lb, confidence);
      if (static_cast<int>(dets.size()) > sample_cap) {
        dets.resize(sample_cap);
      }
      break;
  }
  if (spec_.remove_duplicates) {
    dets = removeDuplicates(std::move(dets), spec_.duplicate_center_distance_px);
    if (static_cast<int>(dets.size()) > max_detections) {
      dets.resize(max_detections);
    }
  }

  Result r;
  r.detections = std::move(dets);
  r.inference_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return r;
}

}  // namespace cobot_detector
