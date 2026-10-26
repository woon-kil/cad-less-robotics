// Copyright 2026 cad-less-robotics contributors
#pragma once

#include "robot_controller/motion_contract.hpp"

#include <array>
#include <string>
#include <vector>

namespace robot_controller {

// 0.5 mm. 웨이포인트 간격 2 mm의 1/4이다. 탐색 결과를 보고 넓히지 않는다.
constexpr double kCapabilityPositionToleranceM = 0.0005;

// full quaternion을 수치적으로 맞춘 것으로 보는 한계. 공정 합격 기준이 아니다.
constexpr double kExactOrientationDeg = 1.0;

// 2 mm 이동에 견주어 한 관절이 이 값보다 크게 움직이면 branch discontinuity다.
constexpr double kBranchDiscontinuityRad = 0.5;

// 위치 Jacobian 영공간 격자. ±0.4 rad, 0.1 rad 간격. 결과 이후에 바꾸지 않는다.
constexpr int kNullHalfCount = 4;
constexpr double kNullStepRad = 0.1;
constexpr int kPositionCorrectionIterations = 12;
constexpr int kCapabilityRuns = 3;

enum class CandidateOrigin {
  PositionSeed,
  NullGrid,
  ExactIk
};

enum class CapabilityClass {
  ExactPose,
  NearPose,
  PositionOnly,
  NoPositionSolution,
  Collision,
  JointLimit
};

struct OrientationMetrics {
  double position_error_m{0.0};
  double tool_axis_error_deg{0.0};
  double roll_error_deg{0.0};
  double full_orientation_error_deg{0.0};
};

struct CapabilityCandidate {
  MotionPose executable;
  std::vector<double> joints;
  OrientationMetrics metrics;
  bool within_limits{false};
  bool colliding{false};
  bool within_position_tolerance{false};
  CandidateOrigin origin{CandidateOrigin::PositionSeed};
};

struct ErrorSummary {
  std::size_t count{0};
  double mean{0.0};
  double p95{0.0};
  double max{0.0};
};

struct CapabilitySelection {
  CapabilityClass capability_class{CapabilityClass::NoPositionSolution};
  bool found{false};
  CapabilityCandidate selected;
  int collision_rejected{0};
  int limit_rejected{0};
  int within_tolerance{0};
  int null_grid_within_tolerance{0};
};

struct JointStep {
  std::vector<double> delta;
  double max_abs{0.0};
  bool branch_discontinuity{false};
};

// desired를 바꾸지 않는다. tool axis를 맞춘 뒤 남은 회전을 roll로 둔다.
OrientationMetrics measureCapability(const MotionPose& desired, const MotionPose& executable);

// 3x5 row-major position Jacobian의 영공간 두 축. 위치 속도가 0인 관절 방향이다.
std::array<std::array<double, 5>, 2> positionNullBasis(const double jacobian_row_major[15]);

// 위치 허용오차 안이고, 한계 안이며, 충돌이 아닌 후보 중 tool-axis 오차가 가장 작은 것을 고른다.
CapabilitySelection selectCapability(const std::vector<CapabilityCandidate>& candidates);

const char* capabilityClassName(CapabilityClass capability_class);

ErrorSummary summarizeErrors(const std::vector<double>& values);

JointStep jointStep(const std::vector<double>& from, const std::vector<double>& to);

}  // namespace robot_controller
