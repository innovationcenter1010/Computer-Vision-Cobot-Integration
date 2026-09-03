#include "cobot_detector/model_registry.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace cobot_detector
{

namespace
{

ModelSpec parseSpec(
  const std::string & section, const std::string & id, const YAML::Node & n,
  const std::string & base_dir)
{
  if (!n.IsMap()) {
    throw std::runtime_error("models.yaml: " + section + "." + id + " must be a mapping");
  }
  ModelSpec s;
  s.id = id;
  s.section = section;

  if (!n["path"]) {
    throw std::runtime_error("models.yaml: " + section + "." + id + " has no 'path'");
  }
  s.path = n["path"].as<std::string>();
  if (!base_dir.empty() && !std::filesystem::path(s.path).is_absolute()) {
    s.path = (std::filesystem::path(base_dir) / s.path).string();
  }

  const std::string type = n["type"] ? n["type"].as<std::string>() : "obb";
  if (type == "obb") {
    s.obb = true;
  } else if (type == "bbox") {
    s.obb = false;
  } else {
    throw std::runtime_error(
            "models.yaml: " + section + "." + id + " type must be 'obb' or 'bbox', got '" +
            type + "'");
  }

  if (n["layout"]) {
    s.layout = n["layout"].as<std::string>();
    if (s.layout != "auto" && s.layout != "raw" && s.layout != "end2end" && s.layout != "seg") {
      throw std::runtime_error(
              "models.yaml: " + section + "." + id + " layout must be auto|raw|end2end|seg, got '" +
              s.layout + "'");
    }
  }

  if (n["confidence"]) {s.confidence = n["confidence"].as<float>();}
  if (n["iou"]) {s.iou = n["iou"].as<float>();}
  if (n["max_detections"]) {s.max_detections = n["max_detections"].as<int>();}
  if (n["remove_duplicates"]) {s.remove_duplicates = n["remove_duplicates"].as<bool>();}
  if (n["duplicate_center_distance_px"]) {
    s.duplicate_center_distance_px = n["duplicate_center_distance_px"].as<float>();
  }

  if (s.confidence <= 0.0f || s.confidence > 1.0f) {
    throw std::runtime_error("models.yaml: " + id + " confidence must be in (0, 1]");
  }
  if (s.iou <= 0.0f || s.iou > 1.0f) {
    throw std::runtime_error("models.yaml: " + id + " iou must be in (0, 1]");
  }
  if (s.max_detections < 1) {
    throw std::runtime_error("models.yaml: " + id + " max_detections must be >= 1");
  }
  return s;
}

}  // namespace

ModelRegistry ModelRegistry::load(const std::string & yaml_path)
{
  std::ifstream in(yaml_path);
  if (!in) {
    throw std::runtime_error("models.yaml not found: " + yaml_path);
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return parse(ss.str(), std::filesystem::path(yaml_path).parent_path().string());
}

ModelRegistry ModelRegistry::parse(const std::string & yaml_text, const std::string & base_dir)
{
  YAML::Node root;
  try {
    root = YAML::Load(yaml_text);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(std::string("models.yaml: parse error: ") + e.what());
  }
  if (!root.IsMap()) {
    throw std::runtime_error("models.yaml: top level must be a mapping");
  }

  ModelRegistry reg;
  for (const char * section : {"runtime", "calibration"}) {
    const YAML::Node sec = root[section];
    if (!sec) {continue;}
    if (!sec.IsMap()) {
      throw std::runtime_error(std::string("models.yaml: '") + section + "' must be a mapping");
    }
    for (const auto & kv : sec) {
      const std::string id = kv.first.as<std::string>();
      if (reg.specs_.count(id)) {
        throw std::runtime_error(
                "models.yaml: model id '" + id + "' declared twice (ids must be unique "
                "across runtime: and calibration:)");
      }
      reg.specs_[id] = parseSpec(section, id, kv.second, base_dir);
    }
  }
  if (reg.specs_.empty()) {
    throw std::runtime_error("models.yaml: no models declared under runtime: or calibration:");
  }
  return reg;
}

const ModelSpec * ModelRegistry::find(const std::string & id) const
{
  const auto it = specs_.find(id);
  return it == specs_.end() ? nullptr : &it->second;
}

std::vector<std::string> ModelRegistry::ids() const
{
  std::vector<std::string> out;
  out.reserve(specs_.size());
  for (const auto & kv : specs_) {out.push_back(kv.first);}
  return out;
}

}  // namespace cobot_detector
