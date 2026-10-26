// Copyright 2026 cad-less-robotics contributors

#include "robot_controller/capability_adapter.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>

namespace {

robot_controller::MotionPose identityPose() {
  return {};
}

robot_controller::MotionPose poseFrom(const Eigen::Quaterniond& rotation, double x) {
  robot_controller::MotionPose pose;
  pose.x = x;
  pose.qx = rotation.x();
  pose.qy = rotation.y();
  pose.qz = rotation.z();
  pose.qw = rotation.w();
  return pose;
}

robot_controller::CapabilityCandidate candidateAt(
    double tool_deg, double full_deg, double position_m, bool colliding, bool within_limits,
    robot_controller::CandidateOrigin origin) {
  robot_controller::CapabilityCandidate candidate;
  candidate.metrics.tool_axis_error_deg = tool_deg;
  candidate.metrics.full_orientation_error_deg = full_deg;
  candidate.metrics.position_error_m = position_m;
  candidate.within_position_tolerance = position_m <= robot_controller::kCapabilityPositionToleranceM;
  candidate.within_limits = within_limits;
  candidate.colliding = colliding;
  candidate.origin = origin;
  candidate.joints = {0.1, 0.2, 0.3, 0.4, 0.5};
  return candidate;
}

}  // namespace

TEST(CapabilityAdapter, LocksThePositionToleranceBeforeAnySearch) {
  EXPECT_DOUBLE_EQ(robot_controller::kCapabilityPositionToleranceM, 0.0005);
  EXPECT_EQ(robot_controller::kNullHalfCount, 4);
  EXPECT_DOUBLE_EQ(robot_controller::kNullStepRad, 0.1);
}

TEST(CapabilityAdapter, DoesNotMutateTheDesiredPose) {
  auto desired = identityPose();
  desired.x = 0.25;
  desired.qw = 1.0;
  const auto copy = desired;
  const auto actual = poseFrom(Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX())), 0.25);
  const auto metrics = robot_controller::measureCapability(desired, actual);
  EXPECT_DOUBLE_EQ(desired.x, copy.x);
  EXPECT_DOUBLE_EQ(desired.qw, copy.qw);
  EXPECT_GT(metrics.tool_axis_error_deg, 1.0);
}

TEST(CapabilityAdapter, SplitsRollFromToolAxis) {
  const auto desired = identityPose();
  constexpr double kDeg = 3.14159265358979323846 / 180.0;
  const auto roll = poseFrom(Eigen::Quaterniond(Eigen::AngleAxisd(20.0 * kDeg, Eigen::Vector3d::UnitZ())), 0.0);
  const auto rolled = robot_controller::measureCapability(desired, roll);
  EXPECT_NEAR(rolled.tool_axis_error_deg, 0.0, 1e-6);
  EXPECT_NEAR(std::abs(rolled.roll_error_deg), 20.0, 1e-4);
  EXPECT_NEAR(rolled.full_orientation_error_deg, 20.0, 1e-4);

  const Eigen::Quaterniond tilted =
      Eigen::Quaterniond(Eigen::AngleAxisd(10.0 * kDeg, Eigen::Vector3d::UnitX())) *
      Eigen::Quaterniond(Eigen::AngleAxisd(20.0 * kDeg, Eigen::Vector3d::UnitZ()));
  const auto mixed = robot_controller::measureCapability(desired, poseFrom(tilted, 0.002));
  EXPECT_NEAR(mixed.position_error_m, 0.002, 1e-12);
  EXPECT_NEAR(mixed.tool_axis_error_deg, 10.0, 1e-3);
  EXPECT_NEAR(std::abs(mixed.roll_error_deg), 20.0, 0.2);
  EXPECT_GT(mixed.full_orientation_error_deg, 10.0);
}

TEST(CapabilityAdapter, RejectsACandidateOutsideThePositionTolerance) {
  auto inside = candidateAt(5.0, 20.0, 0.0005, false, true, robot_controller::CandidateOrigin::NullGrid);
  auto outside = candidateAt(0.1, 0.1, 0.00051, false, true, robot_controller::CandidateOrigin::NullGrid);
  const auto selection = robot_controller::selectCapability({outside, inside});
  EXPECT_EQ(selection.capability_class, robot_controller::CapabilityClass::NearPose);
  EXPECT_NEAR(selection.selected.metrics.tool_axis_error_deg, 5.0, 1e-12);
  EXPECT_EQ(selection.within_tolerance, 1);
}

TEST(CapabilityAdapter, RejectsCollisionAndJointLimitCandidates) {
  auto colliding = candidateAt(1.0, 5.0, 0.0001, true, true, robot_controller::CandidateOrigin::NullGrid);
  auto limited = candidateAt(0.5, 4.0, 0.0001, false, false, robot_controller::CandidateOrigin::PositionSeed);
  auto clear = candidateAt(3.0, 9.0, 0.0002, false, true, robot_controller::CandidateOrigin::NullGrid);
  const auto selection = robot_controller::selectCapability({colliding, limited, clear});
  EXPECT_EQ(selection.capability_class, robot_controller::CapabilityClass::NearPose);
  EXPECT_NEAR(selection.selected.metrics.tool_axis_error_deg, 3.0, 1e-12);
  EXPECT_EQ(selection.collision_rejected, 1);
  EXPECT_EQ(selection.limit_rejected, 1);
}

TEST(CapabilityAdapter, LabelsExactPoseAndPositionOnly) {
  auto exact = candidateAt(0.2, 0.4, 0.0001, false, true, robot_controller::CandidateOrigin::ExactIk);
  EXPECT_EQ(
      robot_controller::selectCapability({exact}).capability_class, robot_controller::CapabilityClass::ExactPose);
  auto seed_only = candidateAt(12.0, 30.0, 0.0001, false, true, robot_controller::CandidateOrigin::PositionSeed);
  EXPECT_EQ(
      robot_controller::selectCapability({seed_only}).capability_class,
      robot_controller::CapabilityClass::PositionOnly);
  EXPECT_EQ(
      robot_controller::selectCapability({}).capability_class, robot_controller::CapabilityClass::NoPositionSolution);
}

TEST(CapabilityAdapter, PrefersSmallerToolAxisThenSmallerRoll) {
  auto worse_tool = candidateAt(8.0, 8.0, 0.0, false, true, robot_controller::CandidateOrigin::NullGrid);
  auto better_tool = candidateAt(2.0, 40.0, 0.0004, false, true, robot_controller::CandidateOrigin::NullGrid);
  auto same_tool_better_full = candidateAt(2.0, 6.0, 0.0004, false, true, robot_controller::CandidateOrigin::NullGrid);
  const auto selection = robot_controller::selectCapability({worse_tool, better_tool, same_tool_better_full});
  EXPECT_NEAR(selection.selected.metrics.full_orientation_error_deg, 6.0, 1e-12);
}

TEST(CapabilityAdapter, CalculatesContinuityAndPercentiles) {
  const auto step = robot_controller::jointStep({0.0, 0.0, 0.0, 0.0, 0.0}, {0.1, -0.2, 0.0, 0.0, 0.6});
  ASSERT_EQ(step.delta.size(), 5u);
  EXPECT_NEAR(step.max_abs, 0.6, 1e-12);
  EXPECT_TRUE(step.branch_discontinuity);
  EXPECT_FALSE(robot_controller::jointStep({0.0}, {0.2}).branch_discontinuity);
  std::vector<double> values;
  for (int index = 0; index <= 10; ++index) {
    values.push_back(static_cast<double>(index));
  }
  const auto summary = robot_controller::summarizeErrors(values);
  EXPECT_NEAR(summary.mean, 5.0, 1e-12);
  EXPECT_NEAR(summary.max, 10.0, 1e-12);
  EXPECT_NEAR(summary.p95, 9.5, 1e-12);
}

TEST(CapabilityAdapter, SealsDesiredAndProjectedSeparately) {
  auto desired = identityPose();
  desired.source_index = 62;
  desired.x = 0.2;
  desired.qw = 1.0;
  const auto before = desired;
  const auto fk = poseFrom(Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX())), 0.2004);
  const auto sealed = robot_controller::sealProjection(desired, fk, {0.1, 0.2, 0.3, 0.4, 0.5}, "NEAR_POSE");
  EXPECT_TRUE(robot_controller::poseFieldsEqual(desired, before));
  EXPECT_TRUE(robot_controller::poseFieldsEqual(sealed.desired, before));
  EXPECT_EQ(sealed.source_index, 62u);
  EXPECT_EQ(sealed.projected.source_index, 62u);
  EXPECT_NEAR(sealed.projected.x, 0.2004, 1e-12);
  EXPECT_GT(std::abs(sealed.projected.qx - desired.qx) + std::abs(sealed.projected.qw - desired.qw), 0.01);
  const auto metrics = robot_controller::measureCapability(before, sealed.projected);
  EXPECT_NEAR(sealed.metrics.position_error_m, metrics.position_error_m, 1e-12);
  EXPECT_NEAR(sealed.metrics.tool_axis_error_deg, metrics.tool_axis_error_deg, 1e-9);
  EXPECT_NEAR(sealed.metrics.roll_error_deg, metrics.roll_error_deg, 1e-9);
  EXPECT_NEAR(sealed.metrics.full_orientation_error_deg, metrics.full_orientation_error_deg, 1e-9);
  const auto targets = robot_controller::projectedPlanningTargets({sealed});
  ASSERT_EQ(targets.size(), 1u);
  EXPECT_TRUE(robot_controller::poseFieldsEqual(targets.front(), sealed.projected));
  EXPECT_FALSE(robot_controller::poseFieldsEqual(targets.front(), sealed.desired));
}

TEST(CapabilityAdapter, PlanningSuccessDoesNotAcceptProcessFidelity) {
  auto desired = identityPose();
  desired.source_index = 63;
  const auto fk = poseFrom(Eigen::Quaterniond(Eigen::AngleAxisd(25.0 * 3.14159265358979323846 / 180.0, Eigen::Vector3d::UnitX())), 0.0);
  const auto sealed = robot_controller::sealProjection(desired, fk, {1.0, 0.0, 0.0, 0.0, 0.0}, "NEAR_POSE");
  const auto before_tool = sealed.metrics.tool_axis_error_deg;
  EXPECT_GT(before_tool, 20.0);
  const auto attached = robot_controller::attachPlanning(sealed, 1.0, 20, true, true, true, robot_controller::kMinCartesianFraction);
  EXPECT_EQ(attached.planning_feasibility, robot_controller::kPlanningFeasible);
  EXPECT_EQ(attached.process_fidelity, robot_controller::kProcessFidelityNotYetAccepted);
  EXPECT_NEAR(attached.projection.metrics.tool_axis_error_deg, before_tool, 1e-12);
  EXPECT_NEAR(sealed.metrics.tool_axis_error_deg, before_tool, 1e-12);
  EXPECT_TRUE(robot_controller::poseFieldsEqual(attached.projection.desired, desired));
  const auto rejected = robot_controller::attachPlanning(sealed, 1.0 / 11.0, 2, true, true, true, robot_controller::kMinCartesianFraction);
  EXPECT_EQ(rejected.planning_feasibility, robot_controller::kPlanningInfeasible);
  EXPECT_EQ(rejected.process_fidelity, robot_controller::kProcessFidelityNotYetAccepted);
  EXPECT_NEAR(rejected.projection.metrics.tool_axis_error_deg, before_tool, 1e-12);
  const auto colliding = robot_controller::attachPlanning(sealed, 1.0, 20, false, true, true, robot_controller::kMinCartesianFraction);
  EXPECT_EQ(colliding.planning_feasibility, robot_controller::kPlanningInfeasible);
}

TEST(CapabilityAdapter, PositionJacobianNullspaceDoesNotMovePosition) {
  double jacobian[15] = {1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0};
  const auto basis = robot_controller::positionNullBasis(jacobian);
  for (const auto& axis : basis) {
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    for (int column = 0; column < 5; ++column) {
      for (int row = 0; row < 3; ++row) {
        velocity(row) += jacobian[(row * 5) + column] * axis[static_cast<std::size_t>(column)];
      }
    }
    EXPECT_LT(velocity.norm(), 1e-9);
  }
  EXPECT_GT(std::abs(basis[0][3]) + std::abs(basis[0][4]) + std::abs(basis[1][3]) + std::abs(basis[1][4]), 0.5);
}
