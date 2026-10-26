// Copyright 2026 cad-less-robotics contributors

#include "robot_controller/capability_adapter.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>

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

bool poseFieldsEqual(const MotionPose& lhs, const MotionPose& rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.qx == rhs.qx && lhs.qy == rhs.qy &&
         lhs.qz == rhs.qz && lhs.qw == rhs.qw && lhs.nx == rhs.nx && lhs.ny == rhs.ny && lhs.nz == rhs.nz &&
         lhs.source_index == rhs.source_index;
}

SealedProjection sealProjection(
    const MotionPose& desired, const MotionPose& fk_pose, const std::vector<double>& joints,
    const char* capability_class) {
  SealedProjection sealed;
  sealed.desired = desired;
  sealed.projected = fk_pose;
  sealed.projected.source_index = desired.source_index;
  sealed.source_index = desired.source_index;
  sealed.joints = joints;
  sealed.capability_class = capability_class == nullptr ? "" : capability_class;
  sealed.metrics = measureCapability(sealed.desired, sealed.projected);
  return sealed;
}

std::vector<MotionPose> projectedPlanningTargets(const std::vector<SealedProjection>& sealed) {
  std::vector<MotionPose> targets;
  targets.reserve(sealed.size());
  for (const auto& item : sealed) {
    targets.push_back(item.projected);
  }
  return targets;
}

PlanningAttachment attachPlanning(
    const SealedProjection& sealed, double fraction, int trajectory_points, bool collision_clear,
    bool limits_clear, bool velocity_clear, double min_fraction) {
  PlanningAttachment attachment;
  attachment.projection = sealed;
  attachment.fraction = fraction;
  attachment.trajectory_points = trajectory_points;
  const bool fraction_ok = fraction + 1e-9 >= min_fraction && trajectory_points >= 2;
  attachment.planning_feasibility =
      (fraction_ok && collision_clear && limits_clear && velocity_clear) ? kPlanningFeasible : kPlanningInfeasible;
  attachment.process_fidelity = kProcessFidelityNotYetAccepted;
  return attachment;
}

bool armJointOrderMatches(const std::vector<std::string>& names) {
  if (names.size() != 5) {
    return false;
  }
  for (std::size_t index = 0; index < 5; ++index) {
    if (names[index] != kArmJointOrder[index]) {
      return false;
    }
  }
  return true;
}

InterpolatedJointPath interpolateJointWaypoints(
    const std::vector<std::vector<double>>& waypoints, double max_step) {
  InterpolatedJointPath path;
  if (waypoints.empty() || !(max_step > 0.0)) {
    return path;
  }
  const std::size_t width = waypoints.front().size();
  path.joints.push_back(waypoints.front());
  path.waypoint_sample_index.push_back(0);
  for (std::size_t segment = 1; segment < waypoints.size(); ++segment) {
    if (waypoints[segment].size() != width) {
      return {};
    }
    double max_delta = 0.0;
    for (std::size_t joint = 0; joint < width; ++joint) {
      max_delta = std::max(max_delta, std::abs(waypoints[segment][joint] - waypoints[segment - 1][joint]));
    }
    const int intervals = std::max(1, static_cast<int>(std::ceil((max_delta / max_step) - 1e-12)));
    for (int step = 1; step <= intervals; ++step) {
      const double blend = static_cast<double>(step) / static_cast<double>(intervals);
      std::vector<double> sample(width);
      for (std::size_t joint = 0; joint < width; ++joint) {
        const double from = waypoints[segment - 1][joint];
        sample[joint] = from + (blend * (waypoints[segment][joint] - from));
      }
      path.joints.push_back(std::move(sample));
    }
    path.waypoint_sample_index.push_back(static_cast<int>(path.joints.size() - 1));
  }
  return path;
}

std::vector<TimedJointPoint> retimeJointPath(
    const std::vector<std::vector<double>>& joints, double max_velocity, double min_dt) {
  std::vector<TimedJointPoint> timed;
  if (joints.empty() || !(max_velocity > 0.0) || !(min_dt > 0.0)) {
    return timed;
  }
  timed.reserve(joints.size());
  double time = 0.0;
  for (std::size_t index = 0; index < joints.size(); ++index) {
    if (index > 0) {
      if (joints[index].size() != joints[index - 1].size()) {
        return {};
      }
      double delta = 0.0;
      for (std::size_t joint = 0; joint < joints[index].size(); ++joint) {
        delta = std::max(delta, std::abs(joints[index][joint] - joints[index - 1][joint]));
      }
      if (!std::isfinite(delta)) {
        return {};
      }
      time += std::max(delta / max_velocity, min_dt);
    }
    timed.push_back(TimedJointPoint{joints[index], time});
  }
  return timed;
}

bool jointVelocityWithin(const std::vector<TimedJointPoint>& timed, double max_velocity) {
  if (timed.size() < 2 || !(max_velocity > 0.0)) {
    return false;
  }
  for (std::size_t index = 1; index < timed.size(); ++index) {
    const double dt = timed[index].time_from_start_sec - timed[index - 1].time_from_start_sec;
    if (!(dt > 1e-4) || timed[index].joints.size() != timed[index - 1].joints.size()) {
      return false;
    }
    double delta = 0.0;
    for (std::size_t joint = 0; joint < timed[index].joints.size(); ++joint) {
      delta = std::max(delta, std::abs(timed[index].joints[joint] - timed[index - 1].joints[joint]));
    }
    if (delta / dt > max_velocity * 1.001) {
      return false;
    }
  }
  return true;
}

double maxFiniteDifferenceAcceleration(const std::vector<TimedJointPoint>& timed) {
  if (timed.size() < 3) {
    return 0.0;
  }
  double max_acceleration = 0.0;
  std::vector<double> previous_velocity(timed[1].joints.size(), 0.0);
  const double first_dt = timed[1].time_from_start_sec - timed[0].time_from_start_sec;
  if (!(first_dt > 1e-4)) {
    return 0.0;
  }
  for (std::size_t joint = 0; joint < previous_velocity.size(); ++joint) {
    previous_velocity[joint] = (timed[1].joints[joint] - timed[0].joints[joint]) / first_dt;
  }
  for (std::size_t index = 2; index < timed.size(); ++index) {
    const double dt = timed[index].time_from_start_sec - timed[index - 1].time_from_start_sec;
    if (!(dt > 1e-4) || timed[index].joints.size() != previous_velocity.size()) {
      continue;
    }
    for (std::size_t joint = 0; joint < previous_velocity.size(); ++joint) {
      const double velocity = (timed[index].joints[joint] - timed[index - 1].joints[joint]) / dt;
      max_acceleration = std::max(max_acceleration, std::abs(velocity - previous_velocity[joint]) / dt);
      previous_velocity[joint] = velocity;
    }
  }
  return max_acceleration;
}

int countOutsideBounds(
    const std::vector<std::vector<double>>& samples, const std::vector<double>& lower,
    const std::vector<double>& upper, double margin) {
  int violations = 0;
  for (const auto& sample : samples) {
    if (sample.size() != lower.size() || sample.size() != upper.size()) {
      ++violations;
      continue;
    }
    for (std::size_t joint = 0; joint < sample.size(); ++joint) {
      if (sample[joint] < lower[joint] - margin || sample[joint] > upper[joint] + margin) {
        ++violations;
        break;
      }
    }
  }
  return violations;
}

std::string classifyJointTrajectory(
    double max_joint_step, int limit_violations, int collision_invalid, bool timing_ok, bool velocity_ok) {
  if (max_joint_step > kBranchDiscontinuityRad) {
    return kContinuityFailure;
  }
  if (limit_violations > 0) {
    return kJointLimitFailure;
  }
  if (collision_invalid > 0) {
    return kCollisionFailure;
  }
  if (!timing_ok) {
    return kTimeParameterizationFailure;
  }
  if (!velocity_ok) {
    return kVelocityFailure;
  }
  return kJointTrajectoryFeasible;
}

std::vector<std::size_t> sourceIndexForSamples(
    const std::vector<std::size_t>& waypoint_sources, const std::vector<int>& waypoint_sample_index,
    std::size_t sample_count) {
  std::vector<std::size_t> sources(sample_count, std::numeric_limits<std::size_t>::max());
  if (waypoint_sources.size() != waypoint_sample_index.size()) {
    return {};
  }
  for (std::size_t index = 0; index < waypoint_sources.size(); ++index) {
    const int sample = waypoint_sample_index[index];
    if (sample < 0 || static_cast<std::size_t>(sample) >= sample_count) {
      return {};
    }
    sources[static_cast<std::size_t>(sample)] = waypoint_sources[index];
  }
  return sources;
}

CartesianSegmentDeviation measureCartesianSegment(const std::vector<MotionPose>& samples) {
  CartesianSegmentDeviation deviation;
  if (samples.size() < 2) {
    return deviation;
  }
  const Eigen::Vector3d start(samples.front().x, samples.front().y, samples.front().z);
  const Eigen::Vector3d end(samples.back().x, samples.back().y, samples.back().z);
  const Eigen::Vector3d chord = end - start;
  const double chord_norm_sq = chord.squaredNorm();
  deviation.chord_length_m = chord.norm();
  for (std::size_t index = 0; index < samples.size(); ++index) {
    if (index > 0) {
      deviation.path_length_m += poseDistance(samples[index - 1], samples[index]);
    }
    const Eigen::Vector3d point(samples[index].x, samples[index].y, samples[index].z);
    double blend = 0.0;
    if (chord_norm_sq > 1e-18) {
      blend = std::clamp((point - start).dot(chord) / chord_norm_sq, 0.0, 1.0);
    }
    deviation.max_chord_deviation_m =
        std::max(deviation.max_chord_deviation_m, (start + (blend * chord) - point).norm());
    deviation.tool_axis_excursion_deg =
        std::max(deviation.tool_axis_excursion_deg, toolAxisAngleDeg(samples.front(), samples[index]));
  }
  return deviation;
}

bool fkMatchesProjectedPose(
    const MotionPose& fk, const MotionPose& projected, double position_tolerance_m,
    double orientation_tolerance_deg) {
  return poseDistance(fk, projected) <= position_tolerance_m &&
         quaternionStepDeg(fk, projected) <= orientation_tolerance_deg;
}

FidelitySeparation separateJointFeasibility(const OrientationMetrics& metrics, const std::string& trajectory_class) {
  FidelitySeparation separation;
  separation.metrics = metrics;
  separation.trajectory_class = trajectory_class;
  separation.process_fidelity = kProcessFidelityNotYetAccepted;
  return separation;
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
