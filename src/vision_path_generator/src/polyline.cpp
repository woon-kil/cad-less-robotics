// Copyright 2026 cad-less-robotics contributors

#include "vision_path_generator/polyline.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace vision_path_generator {
namespace {

bool finite(const Vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

double quaternionDot(const Quat& a, const Quat& b) {
  return (a.x * b.x) + (a.y * b.y) + (a.z * b.z) + (a.w * b.w);
}

}  // namespace

double norm(const Vec3& value) {
  return std::sqrt(dot(value, value));
}

Vec3 normalized(const Vec3& value) {
  const double length = norm(value);
  if (length < 1e-12) {
    return Vec3{};
  }
  return value * (1.0 / length);
}

std::vector<Vec3> resamplePolyline(const std::vector<Vec3>& polyline, double spacing) {
  if (!(spacing > 0.0) || !std::isfinite(spacing)) {
    throw std::invalid_argument("waypoint spacing은 0보다 큰 유한값이어야 합니다.");
  }
  for (const auto& point : polyline) {
    if (!finite(point)) {
      throw std::invalid_argument("폴리라인에 유한하지 않은 점이 있습니다.");
    }
  }
  if (polyline.size() < 2) {
    return polyline;
  }

  std::vector<double> cumulative(polyline.size(), 0.0);
  for (std::size_t index = 1; index < polyline.size(); ++index) {
    cumulative[index] = cumulative[index - 1] + norm(polyline[index] - polyline[index - 1]);
  }
  const double total = cumulative.back();
  if (total < 1e-12) {
    return {polyline.front()};
  }

  const auto steps = static_cast<std::size_t>(std::floor(total / spacing));
  std::vector<Vec3> output;
  output.reserve(steps + 2);
  output.push_back(polyline.front());

  std::size_t segment_index = 1;
  for (std::size_t step = 1; step <= steps; ++step) {
    const double target = spacing * static_cast<double>(step);
    if (target >= total - 1e-9) {
      break;
    }
    while (segment_index + 1 < polyline.size() && cumulative[segment_index] < target) {
      ++segment_index;
    }
    const double span = cumulative[segment_index] - cumulative[segment_index - 1];
    double local = span < 1e-12 ? 0.0 : (target - cumulative[segment_index - 1]) / span;
    local = std::clamp(local, 0.0, 1.0);
    const Vec3 delta = polyline[segment_index] - polyline[segment_index - 1];
    output.push_back(polyline[segment_index - 1] + (delta * local));
  }

  if (norm(output.back() - polyline.back()) > 1e-4) {
    output.push_back(polyline.back());
  }
  return output;
}

std::vector<Vec3> longestNearestNeighborChain(const std::vector<Vec3>& points, double max_gap) {
  if (!(max_gap > 0.0) || !std::isfinite(max_gap)) {
    throw std::invalid_argument("max_gap은 0보다 큰 유한값이어야 합니다.");
  }

  std::vector<Vec3> finite_points;
  finite_points.reserve(points.size());
  for (const auto& point : points) {
    if (finite(point)) {
      finite_points.push_back(point);
    }
  }
  if (finite_points.size() <= 1) {
    return finite_points;
  }

  const std::size_t count = finite_points.size();
  std::vector<char> used(count, 0);
  std::vector<Vec3> best;

  while (true) {
    Vec3 centroid{};
    std::size_t unused = 0;
    for (std::size_t index = 0; index < count; ++index) {
      if (used[index] != 0) {
        continue;
      }
      centroid = centroid + finite_points[index];
      ++unused;
    }
    if (unused == 0) {
      break;
    }
    centroid = centroid * (1.0 / static_cast<double>(unused));

    std::size_t start = 0;
    double farthest = -1.0;
    for (std::size_t index = 0; index < count; ++index) {
      if (used[index] != 0) {
        continue;
      }
      const double distance = norm(finite_points[index] - centroid);
      if (distance > farthest) {
        farthest = distance;
        start = index;
      }
    }

    std::vector<Vec3> chain;
    chain.push_back(finite_points[start]);
    used[start] = 1;
    while (chain.size() < count) {
      const Vec3& current = chain.back();
      double nearest = std::numeric_limits<double>::infinity();
      std::size_t nearest_index = count;
      for (std::size_t index = 0; index < count; ++index) {
        if (used[index] != 0) {
          continue;
        }
        const double distance = norm(finite_points[index] - current);
        if (distance < nearest) {
          nearest = distance;
          nearest_index = index;
        }
      }
      if (nearest_index == count || nearest > max_gap) {
        break;
      }
      used[nearest_index] = 1;
      chain.push_back(finite_points[nearest_index]);
    }
    if (chain.size() > best.size()) {
      best = std::move(chain);
    }
  }
  return best;
}

std::vector<Vec3> smoothOpenPolyline(const std::vector<Vec3>& input, int passes) {
  if (passes < 0) {
    throw std::invalid_argument("smooth_passes는 0 이상이어야 합니다.");
  }
  std::vector<Vec3> current = input;
  for (int pass = 0; pass < passes; ++pass) {
    if (current.size() < 3) {
      break;
    }
    std::vector<Vec3> next = current;
    for (std::size_t index = 1; index + 1 < current.size(); ++index) {
      next[index] = (current[index - 1] + current[index] + current[index + 1]) * (1.0 / 3.0);
    }
    current = std::move(next);
  }
  return current;
}

Vec3 orientNormalTowardsViewpoint(const Vec3& point, const Vec3& normal, const Vec3& viewpoint) {
  if (!finite(point) || !finite(normal) || !finite(viewpoint)) {
    return Vec3{};
  }
  const double length = norm(normal);
  if (length < 1e-9) {
    return Vec3{};
  }
  Vec3 unit = normal * (1.0 / length);
  const Vec3 to_viewpoint = viewpoint - point;
  if (dot(unit, to_viewpoint) < 0.0) {
    unit = unit * -1.0;
  }
  return unit;
}

Quat orientationFromTangentAndNormal(const Vec3& tangent_in, const Vec3& outward_normal_in) {
  Eigen::Vector3d tool_z(outward_normal_in.x, outward_normal_in.y, outward_normal_in.z);
  if (tool_z.norm() < 1e-9) {
    tool_z = Eigen::Vector3d::UnitZ();
  } else {
    tool_z.normalize();
  }
  tool_z = -tool_z;

  Eigen::Vector3d tangent(tangent_in.x, tangent_in.y, tangent_in.z);
  if (tangent.norm() < 1e-9) {
    tangent = Eigen::Vector3d::UnitX();
  } else {
    tangent.normalize();
  }

  Eigen::Vector3d tool_x = tangent - (tool_z * tangent.dot(tool_z));
  if (tool_x.norm() < 1e-6) {
    const Eigen::Vector3d helper = std::abs(tool_z.dot(Eigen::Vector3d::UnitX())) < 0.9
                                       ? Eigen::Vector3d::UnitX()
                                       : Eigen::Vector3d::UnitY();
    tool_x = helper - (tool_z * helper.dot(tool_z));
  }
  tool_x.normalize();
  const Eigen::Vector3d tool_y = tool_z.cross(tool_x).normalized();
  tool_x = tool_y.cross(tool_z).normalized();

  Eigen::Matrix3d rotation;
  rotation.col(0) = tool_x;
  rotation.col(1) = tool_y;
  rotation.col(2) = tool_z;
  Eigen::Quaterniond quaternion(rotation);
  quaternion.normalize();
  return Quat{quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()};
}

Vec3 rotateVector(const Quat& orientation, const Vec3& vector) {
  Eigen::Quaterniond quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
  if (quaternion.norm() < 1e-12) {
    return Vec3{};
  }
  quaternion.normalize();
  const Eigen::Vector3d rotated = quaternion * Eigen::Vector3d(vector.x, vector.y, vector.z);
  return Vec3{rotated.x(), rotated.y(), rotated.z()};
}

void alignQuaternionHemisphere(std::vector<Quat>& orientations) {
  for (std::size_t index = 1; index < orientations.size(); ++index) {
    if (quaternionDot(orientations[index - 1], orientations[index]) < 0.0) {
      orientations[index].x = -orientations[index].x;
      orientations[index].y = -orientations[index].y;
      orientations[index].z = -orientations[index].z;
      orientations[index].w = -orientations[index].w;
    }
  }
}

}  // namespace vision_path_generator
