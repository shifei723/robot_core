#include <gtest/gtest.h>
#include "plan_manage/height_safety.hpp"
using scan_planner::HeightSafety;
TEST(HeightSafety, EmptyObservedFootprintHasNoCeiling) {
  HeightSafety s; EXPECT_TRUE(std::isinf(s.maxBaseHeight({0,0,0.28},0)));
}
TEST(HeightSafety, VoxelExtentAndClearanceAreAccountedFor) {
  HeightSafety s; s.occupied={{0,0,0.625}};
  EXPECT_NEAR(s.maxBaseHeight({0,0,0.28},0),0.242,1e-9);
}
TEST(HeightSafety, WallCannotBePassedByLowering) {
  HeightSafety s; s.occupied={{0,0,0.125},{0,0,0.625}};
  EXPECT_LT(s.maxBaseHeight({0,0,0.28},0),0.1722);
}
TEST(HeightSafety, RearMustClearBeforeStanding) {
  HeightSafety s; s.occupied={{0,-0.3,0.625}};
  EXPECT_NEAR(s.maxBaseHeight({0,0.05,0.2},M_PI/2),0.242,1e-9);
  EXPECT_TRUE(std::isinf(s.maxBaseHeight({0,0.2,0.2},M_PI/2)));
}
TEST(HeightSafety, ForwardSweepDetectsBeforeCurrentFootprint) {
  HeightSafety s; s.occupied={{0.65,0,0.625}};
  EXPECT_TRUE(std::isinf(s.maxBaseHeight({0,0,0.28},0)));
  EXPECT_NEAR(s.sweptMaxBaseHeight({0,0,0.28},0,0.3,0,1.0),0.242,1e-9);
}
TEST(HeightSafety, RotationChecksSweptFootprint) {
  HeightSafety s; s.occupied={{0,0.42,0.125}};
  EXPECT_TRUE(std::isinf(s.maxBaseHeight({0,0,0.28},0)));
  EXPECT_LT(s.sweptMaxBaseHeight({0,0,0.28},0,0,M_PI/2,1.0),0.1722);
}

#include "plan_manage/heading_tracker.hpp"
TEST(HeadingTracker, LongitudinalOvershootDoesNotReverseYaw) {
  const auto c=scan_planner::forwardHeadingCommand({0,1},{0,-0.02},0,0.8,0.35,0);
  EXPECT_DOUBLE_EQ(c.speed,0); EXPECT_NEAR(c.yaw,M_PI/2,1e-9);
}
TEST(HeadingTracker, LateralCorrectionIsBoundedAndForward) {
  const auto c=scan_planner::forwardHeadingCommand({0,1},{0.3,0},0,0.8,0.35,0);
  EXPECT_NEAR(c.yaw,M_PI/2-0.5,1e-9);
}
TEST(HeadingTracker, DegenerateReferenceKeepsHeading) {
  const auto c=scan_planner::forwardHeadingCommand({0,0},{0,-0.02},0,0.8,0.35,1.2);
  EXPECT_DOUBLE_EQ(c.speed,0); EXPECT_DOUBLE_EQ(c.yaw,1.2);
}
