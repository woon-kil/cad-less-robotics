// Copyright 2026 cad-less-robotics contributors
#pragma once

#include "vision_path_generator/polyline.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <string>
#include <vector>

namespace vision_path_generator {

struct ExtractParams {
  float voxel_leaf_size{0.002f};
  int sor_mean_k{30};
  double sor_stddev{1.0};
  double ransac_distance{0.004};
  int ransac_iterations{200};
  double max_plane_inlier_ratio{0.98};
  double min_height{0.003};
  int normal_k{20};
  double boundary_radius{0.01};
  double waypoint_spacing{0.002};
  double max_segment_gap{0.02};
  int smooth_passes{1};
  double cluster_tolerance{0.01};
  int min_cluster_size{50};
  std::size_t min_waypoints{5};
  std::string path_mode{"auto"};
};

struct ExtractResult {
  std::vector<Waypoint> waypoints;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered;
  pcl::PointCloud<pcl::PointXYZ>::Ptr object;
  std::size_t input_points{0};
  std::size_t voxel_points{0};
  std::size_t sor_points{0};
  std::size_t plane_inliers{0};
  std::size_t object_points{0};
  double plane_inlier_ratio{0.0};
  std::string method;
  std::string error;
};

class PathExtractor {
public:
  explicit PathExtractor(ExtractParams params);

  ExtractResult extract(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) const;

private:
  ExtractParams params_;
};

}  // namespace vision_path_generator
