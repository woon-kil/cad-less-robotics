// Copyright 2026 cad-less-robotics contributors
#pragma once

#include <vector>

namespace vision_path_generator {

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quat {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct Waypoint {
  Vec3 position;
  Quat orientation;
  // orientation을 만들 때 쓴 표면 밖 법선. 툴 +Z는 이 벡터의 반대다.
  Vec3 normal;
  // (λ1 - λ0) / λ1. 1에 가까울수록 법선 축이 다음 주성분과 분리된다.
  double normal_separation{0.0};
  bool normal_reliable{false};
  bool normal_recovered{false};
  bool normal_unrecoverable_edge{false};
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) {
  return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3 operator-(const Vec3& a, const Vec3& b) {
  return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3 operator*(const Vec3& value, double scale) {
  return Vec3{value.x * scale, value.y * scale, value.z * scale};
}

inline double dot(const Vec3& a, const Vec3& b) {
  return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

double norm(const Vec3& value);
Vec3 normalized(const Vec3& value);

// 열린 폴리라인을 spacing(미터) 간격으로 다시 뽑는다. 끝점은 항상 포함한다.
std::vector<Vec3> resamplePolyline(const std::vector<Vec3>& polyline, double spacing);

// max_gap보다 먼 점프는 끊고, 가장 긴 사슬만 남긴다.
std::vector<Vec3> longestNearestNeighborChain(const std::vector<Vec3>& points, double max_gap);

std::vector<Vec3> smoothOpenPolyline(const std::vector<Vec3>& input, int passes);

// 툴 +Z는 표면 밖 법선의 반대(소재 쪽)다. 툴 +X는 경로 접선이다.
Quat orientationFromTangentAndNormal(const Vec3& tangent, const Vec3& outward_normal);

// n · (viewpoint - point) >= 0 이 되도록 단위 법선의 부호를 고른다.
// 이웃 법선을 보지 않는다. 법선이 0이거나 값이 유한하지 않으면 0을 반환한다.
Vec3 orientNormalTowardsViewpoint(const Vec3& point, const Vec3& normal, const Vec3& viewpoint);

// λ1이 λ0의 두 배 이상일 때만 평면 법선 축이 분리됐다고 본다.
constexpr double kMinNormalAxisSeparation = 0.5;

double normalAxisSeparation(double lambda0, double lambda1);

enum class NormalRecovery {
  Keep,
  Recovered,
  UnrecoverableEdge,
  UnrecoverableSupport
};

struct RecoveredNormal {
  Vec3 normal;
  NormalRecovery recovery{NormalRecovery::Keep};
};

// 왼쪽·오른쪽 지지 법선이 90° 미만으로 같을 때만 둘의 평균으로 바꾼다.
// 그 경계에서는 original을 유지한다. 이웃 하나를 무조건 복사하지 않는다.
RecoveredNormal recoverNormalFromSupports(
    const Vec3& point,
    const Vec3& original,
    const Vec3& left,
    const Vec3& right,
    bool has_left,
    bool has_right,
    const Vec3& viewpoint);

enum class NormalDiscontinuity {
  None,
  Geometric,
  EstimationOutlier,
  SignAmbiguity,
  PathOrdering,
  Unknown
};

// dot < 0 인 인접 쌍을 부호 오류와 기하 불연속으로 나눈다.
NormalDiscontinuity classifyNormalDiscontinuity(
    const Vec3& previous,
    const Vec3& current,
    const Vec3& next,
    const Vec3& following,
    const Vec3& current_point,
    const Vec3& next_point,
    double segment_length,
    double max_segment_gap,
    bool has_previous,
    bool has_following);

Vec3 rotateVector(const Quat& orientation, const Vec3& vector);
void alignQuaternionHemisphere(std::vector<Quat>& orientations);

}  // namespace vision_path_generator
