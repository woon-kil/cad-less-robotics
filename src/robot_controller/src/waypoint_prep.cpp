// Copyright 2026 cad-less-robotics contributors

#include "robot_controller/waypoint_prep.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace robot_controller {
namespace {

bool finite(double value) {
  return std::isfinite(value);
}

double distance(const Pose& a, const Pose& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
}

double quaternionDot(const Pose& a, const Pose& b) {
  return (a.qx * b.qx) + (a.qy * b.qy) + (a.qz * b.qz) + (a.qw * b.qw);
}

bool normalizeOne(Pose& pose) {
  const double length = std::sqrt(quaternionDot(pose, pose));
  if (!(length > 1e-8) || !std::isfinite(length)) {
    return false;
  }
  pose.qx /= length;
  pose.qy /= length;
  pose.qz /= length;
  pose.qw /= length;
  return true;
}

Eigen::Vector3d toolZ(const Pose& pose) {
  Eigen::Quaterniond quaternion(pose.qw, pose.qx, pose.qy, pose.qz);
  if (quaternion.norm() < 1e-8) {
    throw std::invalid_argument("쿼터니언 크기가 0입니다.");
  }
  quaternion.normalize();
  return quaternion * Eigen::Vector3d::UnitZ();
}

Pose interpolate(const Pose& a, const Pose& b, double t) {
  Pose out;
  out.x = a.x + ((b.x - a.x) * t);
  out.y = a.y + ((b.y - a.y) * t);
  out.z = a.z + ((b.z - a.z) * t);
  const double same_side = quaternionDot(a, b);
  const double sign = same_side < 0.0 ? -1.0 : 1.0;
  out.qx = (a.qx * (1.0 - t)) + (sign * b.qx * t);
  out.qy = (a.qy * (1.0 - t)) + (sign * b.qy * t);
  out.qz = (a.qz * (1.0 - t)) + (sign * b.qz * t);
  out.qw = (a.qw * (1.0 - t)) + (sign * b.qw * t);
  if (!normalizeOne(out)) {
    throw std::invalid_argument("보간된 쿼터니언이 퇴화했습니다.");
  }
  return out;
}

}  // namespace

bool normalizeOrientations(std::vector<Pose>& poses) {
  for (auto& pose : poses) {
    if (!finite(pose.x) || !finite(pose.y) || !finite(pose.z) || !finite(pose.qx) ||
        !finite(pose.qy) || !finite(pose.qz) || !finite(pose.qw)) {
      return false;
    }
    if (!normalizeOne(pose)) {
      return false;
    }
  }
  return true;
}

void alignQuaternionHemisphere(std::vector<Pose>& poses) {
  for (std::size_t index = 1; index < poses.size(); ++index) {
    if (quaternionDot(poses[index - 1], poses[index]) < 0.0) {
      poses[index].qx = -poses[index].qx;
      poses[index].qy = -poses[index].qy;
      poses[index].qz = -poses[index].qz;
      poses[index].qw = -poses[index].qw;
    }
  }
}

std::vector<Pose> dropClosePoses(const std::vector<Pose>& poses, double min_separation) {
  if (!(min_separation >= 0.0) || !std::isfinite(min_separation)) {
    throw std::invalid_argument("min_separation은 0 이상의 유한값이어야 합니다.");
  }
  std::vector<Pose> output;
  output.reserve(poses.size());
  for (const auto& pose : poses) {
    if (!output.empty() && distance(output.back(), pose) < min_separation) {
      continue;
    }
    output.push_back(pose);
  }
  return output;
}

std::vector<Pose> limitWaypointCount(const std::vector<Pose>& poses, std::size_t max_count) {
  if (max_count < 2) {
    throw std::invalid_argument("max_count는 2 이상이어야 합니다.");
  }
  if (poses.size() <= max_count) {
    return poses;
  }

  std::vector<double> cumulative(poses.size(), 0.0);
  for (std::size_t index = 1; index < poses.size(); ++index) {
    cumulative[index] = cumulative[index - 1] + distance(poses[index - 1], poses[index]);
  }
  const double total = cumulative.back();
  if (total < 1e-9) {
    return {poses.front(), poses.back()};
  }

  std::vector<Pose> output;
  output.reserve(max_count);
  for (std::size_t step = 0; step < max_count; ++step) {
    const double target =
        total * (static_cast<double>(step) / static_cast<double>(max_count - 1));
    const auto it = std::lower_bound(cumulative.begin(), cumulative.end(), target);
    std::size_t index = static_cast<std::size_t>(std::distance(cumulative.begin(), it));
    if (index == 0) {
      output.push_back(poses.front());
      continue;
    }
    if (index >= poses.size()) {
      output.push_back(poses.back());
      continue;
    }
    const double span = cumulative[index] - cumulative[index - 1];
    const double local = span < 1e-12 ? 0.0 : (target - cumulative[index - 1]) / span;
    output.push_back(interpolate(poses[index - 1], poses[index], std::clamp(local, 0.0, 1.0)));
  }
  return output;
}

std::vector<Pose> withApproachRetreat(
    const std::vector<Pose>& poses, double approach_m, double retreat_m) {
  if (approach_m < 0.0 || retreat_m < 0.0 || !std::isfinite(approach_m) || !std::isfinite(retreat_m)) {
    throw std::invalid_argument("접근·후퇴 거리는 0 이상의 유한값이어야 합니다.");
  }
  if (poses.empty()) {
    throw std::invalid_argument("접근 포즈를 만들 웨이포인트가 없습니다.");
  }

  std::vector<Pose> output;
  output.reserve(poses.size() + 2);
  if (approach_m > 0.0) {
    const Eigen::Vector3d axis = toolZ(poses.front());
    Pose approach = poses.front();
    approach.x -= axis.x() * approach_m;
    approach.y -= axis.y() * approach_m;
    approach.z -= axis.z() * approach_m;
    output.push_back(approach);
  }
  output.insert(output.end(), poses.begin(), poses.end());
  if (retreat_m > 0.0) {
    const Eigen::Vector3d axis = toolZ(poses.back());
    Pose retreat = poses.back();
    retreat.x -= axis.x() * retreat_m;
    retreat.y -= axis.y() * retreat_m;
    retreat.z -= axis.z() * retreat_m;
    output.push_back(retreat);
  }
  return output;
}

std::string pathError(const std::vector<Pose>& poses, double max_reach_m) {
  if (!(max_reach_m > 0.0) || !std::isfinite(max_reach_m)) {
    return "max_reach_m은 0보다 큰 유한값이어야 합니다.";
  }
  if (poses.size() < 2) {
    return "웨이포인트가 2개 미만입니다.";
  }
  for (const auto& pose : poses) {
    if (!finite(pose.x) || !finite(pose.y) || !finite(pose.z) || !finite(pose.qx) ||
        !finite(pose.qy) || !finite(pose.qz) || !finite(pose.qw)) {
      return "웨이포인트에 유한하지 않은 값이 있습니다.";
    }
    const double qnorm = std::sqrt(quaternionDot(pose, pose));
    if (std::abs(qnorm - 1.0) > 1e-3) {
      return "쿼터니언이 단위 길이가 아닙니다.";
    }
    const double reach = std::sqrt((pose.x * pose.x) + (pose.y * pose.y) + (pose.z * pose.z));
    if (reach > max_reach_m) {
      return "웨이포인트가 도달 거리 한계를 넘습니다.";
    }
  }
  return {};
}

}  // namespace robot_controller
