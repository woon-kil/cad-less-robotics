// Copyright 2026 cad-less-robotics contributors

#include "robot_controller/motion_contract.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace {

robot_controller::MotionPose poseAt(std::size_t index, double x, double yaw_deg) {
  robot_controller::MotionPose pose;
  pose.source_index = index;
  pose.x = x;
  const double half = (yaw_deg * 3.14159265358979323846 / 180.0) * 0.5;
  pose.qy = std::sin(half);
  pose.qw = std::cos(half);
  return pose;
}

std::vector<robot_controller::PairAssessment> pairsFor(const std::vector<robot_controller::MotionPose>& poses) {
  std::vector<robot_controller::PairAssessment> pairs;
  for (std::size_t index = 0; index + 1 < poses.size(); ++index) {
    robot_controller::PairAssessment pair;
    pair.index = index;
    pair.tool_z_deg = robot_controller::toolAxisAngleDeg(poses[index], poses[index + 1]);
    pair.quaternion_deg = robot_controller::quaternionStepDeg(poses[index], poses[index + 1]);
    pair.label = pair.tool_z_deg > 90.0 ? robot_controller::PairLabel::ExplainedGeometric
                                        : robot_controller::PairLabel::Continuous;
    pairs.push_back(pair);
  }
  return pairs;
}

}  // namespace

TEST(MotionContract, AcceptsAFiniteUnitPoseChain) {
  std::vector<robot_controller::MotionPose> poses{poseAt(0, 0.0, 0.0), poseAt(1, 0.002, 10.0)};
  const auto result =
      robot_controller::validateMotionInput(poses, pairsFor(poses), robot_controller::kMaxOrientationStepDeg);
  EXPECT_TRUE(result.error.empty());
  EXPECT_EQ(result.accepted, 2u);
  EXPECT_EQ(result.rejected, 0u);
  EXPECT_EQ(result.segments.size(), 1u);
}

TEST(MotionContract, RejectsANonFinitePosition) {
  auto poses = std::vector<robot_controller::MotionPose>{poseAt(0, 0.0, 0.0), poseAt(1, 0.002, 0.0)};
  poses[1].x = std::nan("");
  const auto result =
      robot_controller::validateMotionInput(poses, pairsFor(poses), robot_controller::kMaxOrientationStepDeg);
  EXPECT_NE(result.error.find("유한"), std::string::npos);
  EXPECT_EQ(result.accepted, 0u);
}

TEST(MotionContract, RejectsADegenerateQuaternion) {
  auto poses = std::vector<robot_controller::MotionPose>{poseAt(0, 0.0, 0.0), poseAt(1, 0.002, 0.0)};
  poses[1].qx = 0.0;
  poses[1].qy = 0.0;
  poses[1].qz = 0.0;
  poses[1].qw = 0.0;
  const auto result =
      robot_controller::validateMotionInput(poses, pairsFor(poses), robot_controller::kMaxOrientationStepDeg);
  EXPECT_NE(result.error.find("쿼터니언"), std::string::npos);
}

TEST(MotionContract, SplitsAnExplainedEdgeWithoutMovingPoints) {
  std::vector<robot_controller::MotionPose> poses{
      poseAt(0, 0.000, 0.0), poseAt(1, 0.002, 0.0), poseAt(2, 0.004, 98.0), poseAt(3, 0.006, 98.0)};
  auto pairs = pairsFor(poses);
  ASSERT_GT(pairs[1].tool_z_deg, 90.0);
  pairs[1].label = robot_controller::PairLabel::ExplainedGeometric;
  const auto result =
      robot_controller::validateMotionInput(poses, pairs, robot_controller::kMaxOrientationStepDeg);
  ASSERT_TRUE(result.error.empty());
  ASSERT_EQ(result.segments.size(), 2u);
  EXPECT_EQ(result.sharp_indices, std::vector<std::size_t>{1});
  EXPECT_EQ(result.segments[0].end_index, 1u);
  EXPECT_EQ(result.segments[1].start_index, 2u);
  EXPECT_EQ(result.segments[0].boundary_after, robot_controller::kBoundaryPolicy);
  EXPECT_EQ(result.segments[0].boundary_reason, "EXPLAINED_GEOMETRIC");
  EXPECT_NEAR(poses[1].x, 0.002, 1e-12);
  EXPECT_NEAR(poses[2].x, 0.004, 1e-12);
}

TEST(MotionContract, RejectsAnUnknownDiscontinuity) {
  std::vector<robot_controller::MotionPose> poses{
      poseAt(0, 0.0, 0.0), poseAt(1, 0.002, 0.0), poseAt(2, 0.004, 98.0)};
  auto pairs = pairsFor(poses);
  pairs[1].label = robot_controller::PairLabel::Invalid;
  const auto result =
      robot_controller::validateMotionInput(poses, pairs, robot_controller::kMaxOrientationStepDeg);
  EXPECT_NE(result.error.find("설명되지 않은"), std::string::npos);
  EXPECT_TRUE(result.segments.empty());
}

TEST(MotionContract, KeepsTheEdgeOutOfBothPlanningWindows) {
  std::vector<robot_controller::MotionPose> poses;
  poses.reserve(56);
  for (std::size_t index = 0; index < 56; ++index) {
    double yaw = 0.0;
    if (index >= 21 && index < 41) {
      yaw = 98.0;
    } else if (index >= 41) {
      yaw = 160.0;
    }
    poses.push_back(poseAt(index, 0.002 * static_cast<double>(index), yaw));
  }
  auto pairs = pairsFor(poses);
  pairs[20].label = robot_controller::PairLabel::ExplainedGeometric;
  const auto validation =
      robot_controller::validateMotionInput(poses, pairs, robot_controller::kMaxOrientationStepDeg);
  ASSERT_TRUE(validation.error.empty()) << validation.error;
  std::string error;
  const auto windows = robot_controller::selectPlanningWindows(
      poses, validation, robot_controller::kPlanningWindowCount, robot_controller::kMaxOrientationStepDeg,
      error);
  ASSERT_TRUE(error.empty()) << error;
  ASSERT_EQ(windows.size(), 3u);
  EXPECT_LE(windows[0].end_index, 20u);
  EXPECT_GE(windows[1].start_index, 21u);
  EXPECT_LE(windows[1].end_index, 40u);
  EXPECT_GE(windows[2].start_index, 41u);
  EXPECT_EQ(windows[0].case_id, "CASE-01");
  EXPECT_EQ(windows[1].case_id, "CASE-02");
  EXPECT_EQ(windows[2].case_id, "CASE-03");
}

TEST(MotionContract, RequiresUncalibratedUnitScaleMetadata) {
  robot_controller::SyntheticPlacement placement;
  placement.source_frame = "camera_depth_optical_frame";
  placement.target_frame = "base_link";
  placement.transform_type = robot_controller::kSyntheticType;
  placement.calibrated = false;
  placement.purpose = "software integration dry-run";
  placement.scale = 1.0;
  EXPECT_TRUE(robot_controller::syntheticMetadataError(placement).empty());
  placement.calibrated = true;
  EXPECT_FALSE(robot_controller::syntheticMetadataError(placement).empty());
  placement.calibrated = false;
  placement.scale = 0.5;
  EXPECT_NE(robot_controller::syntheticMetadataError(placement).find("배율"), std::string::npos);
  placement.scale = 1.0;
  placement.transform_type = "identity";
  EXPECT_FALSE(robot_controller::syntheticMetadataError(placement).empty());
}

TEST(MotionContract, RigidPlacementPreservesRelativeGeometry) {
  const auto start = poseAt(0, 0.10, 20.0);
  const auto next = poseAt(1, 0.12, 25.0);
  robot_controller::MotionPose anchor;
  anchor.x = 0.20;
  anchor.z = 0.15;
  anchor.qw = 1.0;
  const auto placement = robot_controller::rigidPlacement(start, anchor);
  EXPECT_NEAR(placement.scale, 1.0, 1e-15);
  const auto placed_start = robot_controller::applyRigidPlacement(placement, start);
  const auto placed_next = robot_controller::applyRigidPlacement(placement, next);
  EXPECT_NEAR(placed_start.x, anchor.x, 1e-9);
  EXPECT_NEAR(placed_start.y, anchor.y, 1e-9);
  EXPECT_NEAR(placed_start.z, anchor.z, 1e-9);
  EXPECT_NEAR(
      robot_controller::poseDistance(placed_start, placed_next), robot_controller::poseDistance(start, next),
      1e-9);
  EXPECT_NEAR(
      robot_controller::quaternionStepDeg(placed_start, placed_next),
      robot_controller::quaternionStepDeg(start, next), 1e-6);
}
