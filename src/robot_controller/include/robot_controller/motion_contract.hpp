// Copyright 2026 cad-less-robotics contributors
#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace robot_controller {

struct MotionPose {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
  double nx{0.0};
  double ny{0.0};
  double nz{1.0};
  std::size_t source_index{0};
};

// 인접 쌍의 기하 분류. 분류기 결과를 그대로 받는다.
enum class PairLabel {
  Continuous,
  ExplainedGeometric,
  Invalid
};

struct PairAssessment {
  std::size_t index{0};
  double tool_z_deg{0.0};
  double quaternion_deg{0.0};
  PairLabel label{PairLabel::Continuous};
};

// 툴 +Z 각이 이 값을 넘으면 구간 경계다. 45°는 직각의 절반이고, 계획 성공 여부로 바꾸지 않는다.
constexpr double kMaxOrientationStepDeg = 45.0;

// 10걸음 * P-004 간격 0.002 m = 0.020 m.
// DOFBOT 가동 링크 원점 거리 합은 약 0.277 m이므로 이 창은 그 길이의 10% 미만이다.
constexpr std::size_t kPlanningWindowCount = 11;

constexpr char kSyntheticType[] = "TEST_ONLY_SYNTHETIC_PLACEMENT";
constexpr char kBoundaryPolicy[] = "SEGMENT_BOUNDARY_REQUIRES_POLICY";

struct MotionSegment {
  std::size_t start_index{0};
  std::size_t end_index{0};
  std::size_t waypoint_count{0};
  double arc_length_m{0.0};
  std::string boundary_after;
  std::string boundary_reason;
};

struct MotionValidation {
  std::size_t input_count{0};
  std::size_t accepted{0};
  std::size_t rejected{0};
  std::vector<std::size_t> sharp_indices;
  std::vector<MotionSegment> segments;
  std::string error;
};

struct PlanningWindow {
  std::string case_id;
  std::size_t start_index{0};
  std::size_t end_index{0};
  std::size_t waypoint_count{0};
  double arc_length_m{0.0};
};

struct SyntheticPlacement {
  std::string source_frame;
  std::string target_frame;
  std::string transform_type;
  bool calibrated{false};
  std::string purpose;
  double scale{1.0};
};

struct RigidPlacement {
  double rotation[9]{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  double translation[3]{0.0, 0.0, 0.0};
  double scale{1.0};
};

double toolAxisAngleDeg(const MotionPose& a, const MotionPose& b);
double quaternionStepDeg(const MotionPose& a, const MotionPose& b);
double poseDistance(const MotionPose& a, const MotionPose& b);

// 점을 지우거나 법선을 평균하지 않는다. 큰 툴 축 단계는 구간 경계가 된다.
MotionValidation validateMotionInput(
    const std::vector<MotionPose>& poses, const std::vector<PairAssessment>& pairs,
    double max_orientation_step_deg);

// 같은 규칙으로 세 창을 고른다. 플래너 성공 여부는 보지 않는다.
// 1) 가장 큰 툴 축 불연속 앞 구간에서 첫 허용 창
// 2) 그 불연속 뒤 구간에서 첫 허용 창
// 3) 남은 구간 중 가장 긴 곳에서 첫 허용 창
std::vector<PlanningWindow> selectPlanningWindows(
    const std::vector<MotionPose>& poses, const MotionValidation& validation, std::size_t window_count,
    double max_quaternion_step_deg, std::string& error);

std::string syntheticMetadataError(const SyntheticPlacement& placement);

// source_start를 anchor 자세로 보내는 강체 변환. 배율은 1이다.
RigidPlacement rigidPlacement(const MotionPose& source_start, const MotionPose& anchor);
MotionPose applyRigidPlacement(const RigidPlacement& placement, const MotionPose& pose);

}  // namespace robot_controller
