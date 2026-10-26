// Copyright 2026 cad-less-robotics contributors

#include "robot_controller/motion_contract.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_controller {
namespace {

constexpr double kPi = 3.14159265358979323846;

bool finitePose(const MotionPose& pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.z) &&
         std::isfinite(pose.qx) && std::isfinite(pose.qy) && std::isfinite(pose.qz) &&
         std::isfinite(pose.qw);
}

double quaternionNorm(const MotionPose& pose) {
  return std::sqrt(
      (pose.qx * pose.qx) + (pose.qy * pose.qy) + (pose.qz * pose.qz) + (pose.qw * pose.qw));
}

Eigen::Quaterniond quaternionOf(const MotionPose& pose) {
  Eigen::Quaterniond quaternion(pose.qw, pose.qx, pose.qy, pose.qz);
  quaternion.normalize();
  return quaternion;
}

double arcLength(const std::vector<MotionPose>& poses, std::size_t begin, std::size_t end) {
  double length = 0.0;
  for (std::size_t index = begin + 1; index <= end; ++index) {
    length += poseDistance(poses[index - 1], poses[index]);
  }
  return length;
}

bool quaternionWindowOk(
    const std::vector<MotionPose>& poses, std::size_t begin, std::size_t end, double max_step_deg) {
  for (std::size_t index = begin + 1; index <= end; ++index) {
    if (quaternionStepDeg(poses[index - 1], poses[index]) > max_step_deg) {
      return false;
    }
  }
  return true;
}

bool firstAdmissible(
    const std::vector<MotionPose>& poses, const MotionSegment& segment, std::size_t window_count,
    double max_quaternion_step_deg, PlanningWindow& window) {
  if (segment.waypoint_count < window_count) {
    return false;
  }
  const std::size_t last_start = segment.end_index - window_count + 1;
  for (std::size_t start = segment.start_index; start <= last_start; ++start) {
    const std::size_t end = start + window_count - 1;
    if (!quaternionWindowOk(poses, start, end, max_quaternion_step_deg)) {
      continue;
    }
    window.start_index = start;
    window.end_index = end;
    window.waypoint_count = window_count;
    window.arc_length_m = arcLength(poses, start, end);
    return true;
  }
  return false;
}

}  // namespace

double poseDistance(const MotionPose& a, const MotionPose& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
}

double toolAxisAngleDeg(const MotionPose& a, const MotionPose& b) {
  if (quaternionNorm(a) < 1e-8 || quaternionNorm(b) < 1e-8) {
    return 180.0;
  }
  const Eigen::Vector3d first = quaternionOf(a) * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d second = quaternionOf(b) * Eigen::Vector3d::UnitZ();
  const double cosine = std::clamp(first.dot(second), -1.0, 1.0);
  return std::acos(cosine) * (180.0 / kPi);
}

double quaternionStepDeg(const MotionPose& a, const MotionPose& b) {
  if (quaternionNorm(a) < 1e-8 || quaternionNorm(b) < 1e-8) {
    return 180.0;
  }
  const double dot = std::abs(quaternionOf(a).dot(quaternionOf(b)));
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0)) * (180.0 / kPi);
}

MotionValidation validateMotionInput(
    const std::vector<MotionPose>& poses, const std::vector<PairAssessment>& pairs,
    double max_orientation_step_deg) {
  MotionValidation result;
  result.input_count = poses.size();
  if (!(max_orientation_step_deg > 0.0) || !std::isfinite(max_orientation_step_deg)) {
    result.error = "orientation step 한계가 유한한 양수가 아닙니다.";
    result.rejected = poses.size();
    return result;
  }
  if (poses.size() < 2 || pairs.size() + 1 != poses.size()) {
    result.error = "웨이포인트와 인접 쌍 수가 맞지 않습니다.";
    result.rejected = poses.size();
    return result;
  }
  for (std::size_t index = 0; index < poses.size(); ++index) {
    const auto& pose = poses[index];
    if (pose.source_index != index) {
      result.error = "원본 waypoint 순번이 비어 있거나 끊겼습니다.";
      result.rejected = poses.size();
      return result;
    }
    if (!finitePose(pose)) {
      result.error = "웨이포인트에 유한하지 않은 값이 있습니다.";
      result.rejected = poses.size();
      return result;
    }
    const double norm = quaternionNorm(pose);
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) {
      result.error = "쿼터니언이 단위 길이가 아닙니다.";
      result.rejected = poses.size();
      return result;
    }
  }
  for (std::size_t index = 0; index < pairs.size(); ++index) {
    if (pairs[index].index != index) {
      result.error = "인접 쌍 순서가 웨이포인트와 다릅니다.";
      result.rejected = poses.size();
      return result;
    }
    if (pairs[index].label == PairLabel::Invalid) {
      result.error = "설명되지 않은 법선 불연속이 있습니다.";
      result.rejected = poses.size();
      return result;
    }
  }

  std::vector<std::size_t> cuts;
  std::vector<std::string> reasons;
  for (const auto& pair : pairs) {
    if (pair.tool_z_deg > max_orientation_step_deg) {
      cuts.push_back(pair.index);
      reasons.push_back(
          pair.label == PairLabel::ExplainedGeometric ? "EXPLAINED_GEOMETRIC" : "ORIENTATION_STEP");
      result.sharp_indices.push_back(pair.index);
    }
  }

  std::size_t start = 0;
  for (std::size_t cut = 0; cut < cuts.size(); ++cut) {
    MotionSegment segment;
    segment.start_index = poses[start].source_index;
    segment.end_index = poses[cuts[cut]].source_index;
    segment.waypoint_count = cuts[cut] - start + 1;
    segment.arc_length_m = arcLength(poses, start, cuts[cut]);
    segment.boundary_after = kBoundaryPolicy;
    segment.boundary_reason = reasons[cut];
    result.segments.push_back(segment);
    start = cuts[cut] + 1;
  }
  MotionSegment tail;
  tail.start_index = poses[start].source_index;
  tail.end_index = poses.back().source_index;
  tail.waypoint_count = poses.size() - start;
  tail.arc_length_m = arcLength(poses, start, poses.size() - 1);
  result.segments.push_back(tail);
  result.accepted = poses.size();
  return result;
}

std::vector<PlanningWindow> selectPlanningWindows(
    const std::vector<MotionPose>& poses, const MotionValidation& validation, std::size_t window_count,
    double max_quaternion_step_deg, std::string& error) {
  error.clear();
  if (!validation.error.empty()) {
    error = validation.error;
    return {};
  }
  if (window_count < 2 || validation.sharp_indices.empty() || validation.segments.size() < 2) {
    error = "계획 창을 만들 구간 경계가 없습니다.";
    return {};
  }
  std::size_t primary = validation.sharp_indices.front();
  double primary_angle = -1.0;
  for (const auto index : validation.sharp_indices) {
    const double angle = toolAxisAngleDeg(poses[index], poses[index + 1]);
    if (angle > primary_angle) {
      primary_angle = angle;
      primary = index;
    }
  }
  const MotionSegment* before = nullptr;
  const MotionSegment* after = nullptr;
  for (const auto& segment : validation.segments) {
    if (segment.end_index == poses[primary].source_index) {
      before = &segment;
    }
    if (segment.start_index == poses[primary + 1].source_index) {
      after = &segment;
    }
  }
  if (before == nullptr || after == nullptr) {
    error = "주 불연속의 앞뒤 구간을 찾지 못했습니다.";
    return {};
  }

  std::vector<PlanningWindow> windows(3);
  windows[0].case_id = "CASE-01";
  windows[1].case_id = "CASE-02";
  windows[2].case_id = "CASE-03";
  if (!firstAdmissible(poses, *before, window_count, max_quaternion_step_deg, windows[0]) ||
      !firstAdmissible(poses, *after, window_count, max_quaternion_step_deg, windows[1])) {
    error = "주 불연속 앞뒤에서 자세 단계가 허용되는 창이 없습니다.";
    return {};
  }

  const MotionSegment* longest = nullptr;
  for (const auto& segment : validation.segments) {
    if (&segment == before || &segment == after) {
      continue;
    }
    if (longest == nullptr || segment.waypoint_count > longest->waypoint_count) {
      longest = &segment;
    }
  }
  if (longest == nullptr ||
      !firstAdmissible(poses, *longest, window_count, max_quaternion_step_deg, windows[2])) {
    error = "세 번째 연속 창이 없습니다.";
    return {};
  }
  return windows;
}

std::string syntheticMetadataError(const SyntheticPlacement& placement) {
  if (placement.transform_type != kSyntheticType) {
    return "배치 변환 종류가 TEST_ONLY_SYNTHETIC_PLACEMENT가 아닙니다.";
  }
  if (placement.calibrated) {
    return "synthetic placement를 calibration으로 표시할 수 없습니다.";
  }
  if (placement.source_frame.empty() || placement.target_frame.empty()) {
    return "배치 변환의 프레임이 비어 있습니다.";
  }
  if (placement.source_frame == placement.target_frame) {
    return "카메라 프레임과 계획 프레임을 같은 프레임으로 두지 않습니다.";
  }
  if (!std::isfinite(placement.scale) || std::abs(placement.scale - 1.0) > 1e-12) {
    return "배율은 1이어야 합니다.";
  }
  if (placement.purpose.empty()) {
    return "배치 변환의 목적이 비어 있습니다.";
  }
  return {};
}

RigidPlacement rigidPlacement(const MotionPose& source_start, const MotionPose& anchor) {
  const Eigen::Quaterniond rotation = quaternionOf(anchor) * quaternionOf(source_start).inverse();
  const Eigen::Vector3d source(source_start.x, source_start.y, source_start.z);
  const Eigen::Vector3d target(anchor.x, anchor.y, anchor.z);
  const Eigen::Vector3d translation = target - (rotation * source);
  const Eigen::Matrix3d matrix = rotation.toRotationMatrix();
  RigidPlacement placement;
  for (int row = 0; row < 3; ++row) {
    placement.translation[row] = translation(row);
    for (int column = 0; column < 3; ++column) {
      placement.rotation[(row * 3) + column] = matrix(row, column);
    }
  }
  placement.scale = 1.0;
  return placement;
}

MotionPose applyRigidPlacement(const RigidPlacement& placement, const MotionPose& pose) {
  if (std::abs(placement.scale - 1.0) > 1e-12) {
    throw std::invalid_argument("강체 배치의 배율은 1이어야 합니다.");
  }
  Eigen::Matrix3d matrix;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      matrix(row, column) = placement.rotation[(row * 3) + column];
    }
  }
  const Eigen::Vector3d translated =
      (matrix * Eigen::Vector3d(pose.x, pose.y, pose.z)) +
      Eigen::Vector3d(placement.translation[0], placement.translation[1], placement.translation[2]);
  const Eigen::Quaterniond rotated(matrix);
  const Eigen::Quaterniond orientation = rotated * quaternionOf(pose);
  MotionPose output = pose;
  output.x = translated.x();
  output.y = translated.y();
  output.z = translated.z();
  output.qx = orientation.x();
  output.qy = orientation.y();
  output.qz = orientation.z();
  output.qw = orientation.w();
  const Eigen::Vector3d normal = matrix * Eigen::Vector3d(pose.nx, pose.ny, pose.nz);
  output.nx = normal.x();
  output.ny = normal.y();
  output.nz = normal.z();
  return output;
}

}  // namespace robot_controller
