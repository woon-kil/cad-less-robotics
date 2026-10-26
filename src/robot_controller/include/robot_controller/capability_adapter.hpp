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

// Attempt #1과 같은 Cartesian 합격 분율. 이번 단계에서 내리지 않는다.
constexpr double kMinCartesianFraction = 0.95;

constexpr char kPlanningFeasible[] = "PLANNING_FEASIBLE";
constexpr char kPlanningInfeasible[] = "PLANNING_INFEASIBLE";
constexpr char kProcessFidelityNotYetAccepted[] = "PROCESS_FIDELITY_NOT_YET_ACCEPTED";

struct SealedProjection {
  MotionPose desired;
  MotionPose projected;
  OrientationMetrics metrics;
  std::size_t source_index{0};
  std::vector<double> joints;
  std::string capability_class;
};

struct PlanningAttachment {
  SealedProjection projection;
  double fraction{0.0};
  int trajectory_points{0};
  std::string planning_feasibility;
  std::string process_fidelity;
};

bool poseFieldsEqual(const MotionPose& lhs, const MotionPose& rhs);

// FK로 만든 projected pose를 desired와 분리해 봉인한다. desired 인자는 바꾸지 않는다.
SealedProjection sealProjection(
    const MotionPose& desired, const MotionPose& fk_pose, const std::vector<double>& joints,
    const char* capability_class);

std::vector<MotionPose> projectedPlanningTargets(const std::vector<SealedProjection>& sealed);

// fraction을 적어도 fidelity metric과 상태 문자열은 바꾸지 않는다.
PlanningAttachment attachPlanning(
    const SealedProjection& sealed, double fraction, int trajectory_points, bool collision_clear,
    bool limits_clear, bool velocity_clear, double min_fraction);

// 0.05 rad. 0.5 rad branch 문턱의 1/10이고, Attempt #3 최대 관절 간격 0.4 rad를 8구간으로 나눈다.
// Attempt #5 결과를 보고 바꾸지 않는다.
constexpr double kMaxJointInterpolationStepRad = 0.05;

// motion_planner_node::retime과 같다. 새 속도 스케줄러가 아니다.
constexpr double kJointVelocityCapRadS = 0.4;
constexpr double kJointTimeStepFloorSec = 0.05;

constexpr char kJointTrajectoryFeasible[] = "JOINT_TRAJECTORY_FEASIBLE";
constexpr char kJointLimitFailure[] = "JOINT_LIMIT_FAILURE";
constexpr char kCollisionFailure[] = "COLLISION_FAILURE";
constexpr char kVelocityFailure[] = "VELOCITY_FAILURE";
constexpr char kContinuityFailure[] = "CONTINUITY_FAILURE";
constexpr char kTimeParameterizationFailure[] = "TIME_PARAMETERIZATION_FAILURE";

inline constexpr const char* kArmJointOrder[] = {
    "Arm1_Joint", "Arm2_Joint", "Arm3_Joint", "Arm4_Joint", "Arm5_Joint"};

struct InterpolatedJointPath {
  std::vector<std::vector<double>> joints;
  std::vector<int> waypoint_sample_index;
};

struct TimedJointPoint {
  std::vector<double> joints;
  double time_from_start_sec{0.0};
};

struct CartesianSegmentDeviation {
  double path_length_m{0.0};
  double chord_length_m{0.0};
  double max_chord_deviation_m{0.0};
  double tool_axis_excursion_deg{0.0};
};

struct FidelitySeparation {
  OrientationMetrics metrics;
  std::string trajectory_class;
  std::string process_fidelity;
};

bool armJointOrderMatches(const std::vector<std::string>& names);

// q를 다시 IK하지 않는다. 양 끝 waypoint를 포함하고 구간 최대 관절 간격은 max_step 이하다.
InterpolatedJointPath interpolateJointWaypoints(
    const std::vector<std::vector<double>>& waypoints, double max_step);

std::vector<TimedJointPoint> retimeJointPath(
    const std::vector<std::vector<double>>& joints, double max_velocity, double min_dt);

bool jointVelocityWithin(const std::vector<TimedJointPoint>& timed, double max_velocity);

double maxFiniteDifferenceAcceleration(const std::vector<TimedJointPoint>& timed);

int countOutsideBounds(
    const std::vector<std::vector<double>>& samples, const std::vector<double>& lower,
    const std::vector<double>& upper, double margin);

std::string classifyJointTrajectory(
    double max_joint_step, int limit_violations, int collision_invalid, bool timing_ok, bool velocity_ok);

std::vector<std::size_t> sourceIndexForSamples(
    const std::vector<std::size_t>& waypoint_sources, const std::vector<int>& waypoint_sample_index,
    std::size_t sample_count);

CartesianSegmentDeviation measureCartesianSegment(const std::vector<MotionPose>& samples);

bool fkMatchesProjectedPose(
    const MotionPose& fk, const MotionPose& projected, double position_tolerance_m,
    double orientation_tolerance_deg);

FidelitySeparation separateJointFeasibility(const OrientationMetrics& metrics, const std::string& trajectory_class);

}  // namespace robot_controller
