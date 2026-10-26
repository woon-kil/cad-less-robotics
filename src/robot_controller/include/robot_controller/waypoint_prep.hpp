// Copyright 2026 cad-less-robotics contributors
#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace robot_controller {

struct Pose {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

bool normalizeOrientations(std::vector<Pose>& poses);
void alignQuaternionHemisphere(std::vector<Pose>& poses);
std::vector<Pose> dropClosePoses(const std::vector<Pose>& poses, double min_separation);
std::vector<Pose> limitWaypointCount(const std::vector<Pose>& poses, std::size_t max_count);
std::vector<Pose> withApproachRetreat(
  const std::vector<Pose>& poses, double approach_m, double retreat_m);
std::string pathError(const std::vector<Pose>& poses, double max_reach_m);

}  // namespace robot_controller
