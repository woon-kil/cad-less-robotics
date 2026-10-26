// Copyright 2026 cad-less-robotics contributors

#include "robot_controller/capability_adapter.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace robot_controller {
namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Quaterniond rotationOf(const MotionPose& pose) {
  Eigen::Quaterniond rotation(pose.qw, pose.qx, pose.qy, pose.qz);
  rotation.normalize();
  return rotation;
}

double wrapDegrees(double degrees) {
  while (degrees > 180.0) {
    degrees -= 360.0;
  }
  while (degrees < -180.0) {
    degrees += 360.0;
  }
  return degrees;
}

bool betterToolAxis(const CapabilityCandidate& lhs, const CapabilityCandidate& rhs) {
  if (lhs.metrics.tool_axis_error_deg + 1e-9 < rhs.metrics.tool_axis_error_deg) {
    return true;
  }
  if (rhs.metrics.tool_axis_error_deg + 1e-9 < lhs.metrics.tool_axis_error_deg) {
    return false;
  }
  if (lhs.metrics.full_orientation_error_deg + 1e-9 < rhs.metrics.full_orientation_error_deg) {
    return true;
  }
  if (rhs.metrics.full_orientation_error_deg + 1e-9 < lhs.metrics.full_orientation_error_deg) {
    return false;
  }
  return lhs.metrics.position_error_m < rhs.metrics.position_error_m;
}

}  // namespace

OrientationMetrics measureCapability(const MotionPose& desired, const MotionPose& executable) {
  OrientationMetrics metrics;
  metrics.position_error_m = poseDistance(desired, executable);
  metrics.tool_axis_error_deg = toolAxisAngleDeg(desired, executable);
  metrics.full_orientation_error_deg = quaternionStepDeg(desired, executable);
  const Eigen::Quaterniond desired_q = rotationOf(desired);
  const Eigen::Quaterniond executable_q = rotationOf(executable);
  const Eigen::Vector3d desired_axis = desired_q * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d executable_axis = executable_q * Eigen::Vector3d::UnitZ();
  const Eigen::Quaterniond aligned =
      Eigen::Quaterniond::FromTwoVectors(executable_axis, desired_axis) * executable_q;
  Eigen::Quaterniond roll = desired_q.conjugate() * aligned;
  if (roll.w() < 0.0) {
    roll.coeffs() *= -1.0;
  }
  const Eigen::AngleAxisd axis(roll);
  metrics.roll_error_deg = wrapDegrees(axis.angle() * axis.axis().z() * 180.0 / kPi);
  return metrics;
}

std::array<std::array<double, 5>, 2> positionNullBasis(const double jacobian_row_major[15]) {
  Eigen::Matrix<double, 3, 5> jacobian;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 5; ++column) {
      jacobian(row, column) = jacobian_row_major[(row * 5) + column];
    }
  }
  Eigen::JacobiSVD<Eigen::Matrix<double, 3, 5>> svd(jacobian, Eigen::ComputeFullV);
  std::array<std::array<double, 5>, 2> basis{};
  const Eigen::Matrix<double, 5, 5>& vectors = svd.matrixV();
  for (int axis = 0; axis < 2; ++axis) {
    const auto column = vectors.col(3 + axis);
    for (int joint = 0; joint < 5; ++joint) {
      basis[static_cast<std::size_t>(axis)][static_cast<std::size_t>(joint)] = column(joint);
    }
  }
  return basis;
}

CapabilitySelection selectCapability(const std::vector<CapabilityCandidate>& candidates) {
  CapabilitySelection selection;
  bool have_in_limit = false;
  bool have_collision = false;
  bool have_limit_failure = false;
  for (const auto& candidate : candidates) {
    if (!candidate.within_position_tolerance ||
        !(candidate.metrics.position_error_m <= kCapabilityPositionToleranceM)) {
      continue;
    }
    ++selection.within_tolerance;
    if (candidate.origin == CandidateOrigin::NullGrid) {
      ++selection.null_grid_within_tolerance;
    }
    if (!candidate.within_limits) {
      ++selection.limit_rejected;
      have_limit_failure = true;
      continue;
    }
    have_in_limit = true;
    if (candidate.colliding) {
      ++selection.collision_rejected;
      have_collision = true;
      continue;
    }
    if (!selection.found || betterToolAxis(candidate, selection.selected)) {
      selection.selected = candidate;
      selection.found = true;
    }
  }
  if (!selection.found) {
    if (have_in_limit && have_collision) {
      selection.capability_class = CapabilityClass::Collision;
    } else if (have_limit_failure) {
      selection.capability_class = CapabilityClass::JointLimit;
    } else {
      selection.capability_class = CapabilityClass::NoPositionSolution;
    }
    return selection;
  }
  const bool exact_ik = selection.selected.origin == CandidateOrigin::ExactIk &&
                        selection.selected.metrics.full_orientation_error_deg <= kExactOrientationDeg;
  if (exact_ik) {
    selection.capability_class = CapabilityClass::ExactPose;
  } else if (selection.null_grid_within_tolerance > 0 ||
             selection.selected.origin == CandidateOrigin::NullGrid ||
             selection.selected.origin == CandidateOrigin::ExactIk) {
    selection.capability_class = CapabilityClass::NearPose;
  } else {
    selection.capability_class = CapabilityClass::PositionOnly;
  }
  return selection;
}

const char* capabilityClassName(CapabilityClass capability_class) {
  switch (capability_class) {
    case CapabilityClass::ExactPose:
      return "EXACT_POSE";
    case CapabilityClass::NearPose:
      return "NEAR_POSE";
    case CapabilityClass::PositionOnly:
      return "POSITION_ONLY";
    case CapabilityClass::NoPositionSolution:
      return "NO_POSITION_SOLUTION";
    case CapabilityClass::Collision:
      return "COLLISION";
    case CapabilityClass::JointLimit:
      return "JOINT_LIMIT";
  }
  return "NO_POSITION_SOLUTION";
}

ErrorSummary summarizeErrors(const std::vector<double>& values) {
  ErrorSummary summary;
  summary.count = values.size();
  if (values.empty()) {
    return summary;
  }
  double total = 0.0;
  for (double value : values) {
    total += value;
    summary.max = std::max(summary.max, value);
  }
  summary.mean = total / static_cast<double>(values.size());
  std::vector<double> sorted = values;
  std::sort(sorted.begin(), sorted.end());
  const double rank = 0.95 * static_cast<double>(sorted.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(rank));
  const auto upper = static_cast<std::size_t>(std::ceil(rank));
  const double fraction = rank - static_cast<double>(lower);
  summary.p95 = ((1.0 - fraction) * sorted[lower]) + (fraction * sorted[upper]);
  return summary;
}

JointStep jointStep(const std::vector<double>& from, const std::vector<double>& to) {
  JointStep step;
  if (from.size() != to.size() || from.empty()) {
    return step;
  }
  step.delta.resize(from.size());
  for (std::size_t index = 0; index < from.size(); ++index) {
    step.delta[index] = to[index] - from[index];
    step.max_abs = std::max(step.max_abs, std::abs(step.delta[index]));
  }
  step.branch_discontinuity = step.max_abs > kBranchDiscontinuityRad;
  return step;
}

}  // namespace robot_controller
