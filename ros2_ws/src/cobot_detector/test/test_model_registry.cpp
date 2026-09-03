#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "cobot_detector/model_registry.hpp"

using cobot_detector::ModelRegistry;

TEST(ModelRegistry, ParsesBothSectionsAndDefaults)
{
  const std::string yaml = R"(
runtime:
  cans:
    path: cans/cans.onnx
    type: obb
    confidence: 0.6
    remove_duplicates: true
calibration:
  red_dot:
    path: /abs/red_dot.onnx
    type: bbox
)";
  const auto reg = ModelRegistry::parse(yaml, "/ws/models");
  ASSERT_EQ(reg.all().size(), 2u);

  const auto * cans = reg.find("cans");
  ASSERT_NE(cans, nullptr);
  EXPECT_EQ(cans->section, "runtime");
  EXPECT_EQ(cans->path, "/ws/models/cans/cans.onnx");   // relative → beside the yaml
  EXPECT_TRUE(cans->obb);
  EXPECT_FLOAT_EQ(cans->confidence, 0.6f);
  EXPECT_FLOAT_EQ(cans->iou, 0.4f);                     // default
  EXPECT_EQ(cans->max_detections, 20);                  // default
  EXPECT_TRUE(cans->remove_duplicates);

  const auto * dot = reg.find("red_dot");
  ASSERT_NE(dot, nullptr);
  EXPECT_EQ(dot->section, "calibration");
  EXPECT_EQ(dot->path, "/abs/red_dot.onnx");            // absolute kept
  EXPECT_FALSE(dot->obb);
  EXPECT_FALSE(dot->remove_duplicates);

  EXPECT_EQ(reg.find("nope"), nullptr);
}

TEST(ModelRegistry, ParsesLayoutKey)
{
  const auto reg = ModelRegistry::parse(
    "runtime:\n  a: {path: a.onnx, layout: end2end}\n  b: {path: b.onnx}\n");
  EXPECT_EQ(reg.find("a")->layout, "end2end");
  EXPECT_EQ(reg.find("b")->layout, "auto");
  EXPECT_THROW(ModelRegistry::parse("runtime:\n  x: {path: a.onnx, layout: nms}\n"),
    std::runtime_error);
}

TEST(ModelRegistry, RejectsDuplicateIdsAcrossSections)
{
  const std::string yaml = R"(
runtime:
  x: {path: a.onnx}
calibration:
  x: {path: b.onnx}
)";
  EXPECT_THROW(ModelRegistry::parse(yaml), std::runtime_error);
}

TEST(ModelRegistry, RejectsBadTypeMissingPathAndEmpty)
{
  EXPECT_THROW(ModelRegistry::parse("runtime:\n  x: {path: a.onnx, type: seg}\n"), std::runtime_error);
  EXPECT_THROW(ModelRegistry::parse("runtime:\n  x: {type: obb}\n"), std::runtime_error);
  EXPECT_THROW(ModelRegistry::parse("other:\n  x: {path: a.onnx}\n"), std::runtime_error);
  EXPECT_THROW(ModelRegistry::parse("runtime:\n  x: {path: a.onnx, confidence: 1.5}\n"), std::runtime_error);
}

TEST(ModelRegistry, LoadsShippedExample)
{
  const auto reg = ModelRegistry::load(std::string(TEST_DATA_DIR) + "/models.yaml");
  EXPECT_NE(reg.find("cans"), nullptr);
  EXPECT_NE(reg.find("red_dot"), nullptr);
}
