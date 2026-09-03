#include <gtest/gtest.h>

#include "cobot_control/motoman_robot.hpp"

using namespace cobot_control;

TEST(NormalizeDeg180, InRangeValuesAreUnchanged)
{
  EXPECT_DOUBLE_EQ(normalizeDeg180(0.0), 0.0);
  EXPECT_DOUBLE_EQ(normalizeDeg180(90.0), 90.0);
  EXPECT_DOUBLE_EQ(normalizeDeg180(-90.0), -90.0);
  EXPECT_DOUBLE_EQ(normalizeDeg180(-179.0), -179.0);
}

TEST(NormalizeDeg180, BoundaryWrapsToNegative180)
{
  EXPECT_DOUBLE_EQ(normalizeDeg180(180.0), -180.0);
  EXPECT_DOUBLE_EQ(normalizeDeg180(-180.0), -180.0);
}

TEST(NormalizeDeg180, JustOverBoundaryWraps)
{
  EXPECT_DOUBLE_EQ(normalizeDeg180(181.0), -179.0);
  EXPECT_DOUBLE_EQ(normalizeDeg180(-181.0), 179.0);
}

TEST(NormalizeDeg180, MultipleWrapsAroundHandled)
{
  EXPECT_DOUBLE_EQ(normalizeDeg180(270.0), -90.0);
  EXPECT_DOUBLE_EQ(normalizeDeg180(450.0), 90.0);
  EXPECT_DOUBLE_EQ(normalizeDeg180(-450.0), -90.0);
  EXPECT_DOUBLE_EQ(normalizeDeg180(720.0), 0.0);
}

// The motivating case: a taught rz plus the mount_yaw_offset_deg correction
// pushing past +/-180 must land back at the equivalent in-range angle, not
// an out-of-convention value like 190.
TEST(NormalizeDeg180, TeachPlusMountOffsetStaysInRange)
{
  const double taught_rz = 100.0;
  const double mount_yaw_offset_deg = 90.0;
  EXPECT_DOUBLE_EQ(normalizeDeg180(taught_rz + mount_yaw_offset_deg), -170.0);
}
