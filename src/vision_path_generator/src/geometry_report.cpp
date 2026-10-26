// Copyright 2026 cad-less-robotics contributors

#include "vision_path_generator/path_extractor.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr double kTargetSpacing = 0.002;
constexpr double kSpacingTolerance = 0.00025;
constexpr double kPi = 3.14159265358979323846;

struct SpacingStats {
  std::size_t internal_count{0};
  double mean{0.0};
  double stddev{0.0};
  double min{0.0};
  double max{0.0};
  double mean_abs_error{0.0};
  double max_abs_error{0.0};
  bool has_remainder{false};
  double remainder{0.0};
  std::size_t outlier_index{0};
  double outlier_gap{0.0};
};

struct OrientationStats {
  std::size_t invalid_quaternion{0};
  std::size_t zero_quaternion{0};
  std::size_t bad_norm{0};
  std::size_t non_finite{0};
  std::size_t quaternion_flips{0};
  std::size_t normal_flips{0};
  std::size_t hemisphere_violations{0};
  double mean_error_deg{0.0};
  double p95_error_deg{0.0};
  double max_error_deg{0.0};
  bool recorded{false};
  double min_adjacent_dot{0.0};
  double mean_adjacent_dot{0.0};
  double p05_adjacent_dot{0.0};
  double max_adjacent_angle_deg{0.0};
};

std::map<std::string, std::string> readMeta(const std::string& path) {
  std::ifstream input(path);
  std::map<std::string, std::string> values;
  std::string line;
  while (std::getline(input, line)) {
    const auto split = line.find('=');
    if (split == std::string::npos || line.empty() || line[0] == '#') {
      continue;
    }
    values[line.substr(0, split)] = line.substr(split + 1);
  }
  return values;
}

std::string jsonEscape(const std::string& value) {
  std::string output;
  for (const char ch : value) {
    if (ch == '\\' || ch == '"') {
      output.push_back('\\');
    }
    output.push_back(ch);
  }
  return output;
}

std::string jsonNumber(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream stream;
  stream << std::setprecision(17) << value;
  return stream.str();
}

double quatNorm(const vision_path_generator::Quat& quat) {
  return std::sqrt((quat.x * quat.x) + (quat.y * quat.y) + (quat.z * quat.z) + (quat.w * quat.w));
}

double quatDot(const vision_path_generator::Quat& a, const vision_path_generator::Quat& b) {
  return (a.x * b.x) + (a.y * b.y) + (a.z * b.z) + (a.w * b.w);
}

bool sameWaypoint(const vision_path_generator::Waypoint& a, const vision_path_generator::Waypoint& b) {
  return a.position.x == b.position.x && a.position.y == b.position.y && a.position.z == b.position.z &&
         a.orientation.x == b.orientation.x && a.orientation.y == b.orientation.y &&
         a.orientation.z == b.orientation.z && a.orientation.w == b.orientation.w &&
         a.normal.x == b.normal.x && a.normal.y == b.normal.y && a.normal.z == b.normal.z;
}

bool sameRun(
    const vision_path_generator::ExtractResult& a, const vision_path_generator::ExtractResult& b) {
  if (a.waypoints.size() != b.waypoints.size() || a.raw_path_points != b.raw_path_points ||
      a.raw_path_length != b.raw_path_length || a.method != b.method || a.error != b.error ||
      a.plane_inliers != b.plane_inliers || a.object_points != b.object_points) {
    return false;
  }
  for (std::size_t index = 0; index < a.waypoints.size(); ++index) {
    if (!sameWaypoint(a.waypoints[index], b.waypoints[index])) {
      return false;
    }
  }
  return true;
}

SpacingStats spacingOf(const std::vector<vision_path_generator::Waypoint>& waypoints) {
  SpacingStats stats;
  if (waypoints.size() < 2) {
    return stats;
  }
  std::vector<double> gaps;
  gaps.reserve(waypoints.size() - 1);
  for (std::size_t index = 1; index < waypoints.size(); ++index) {
    gaps.push_back(vision_path_generator::norm(waypoints[index].position - waypoints[index - 1].position));
  }
  std::vector<double> internal = gaps;
  if (!internal.empty()) {
    stats.has_remainder = true;
    stats.remainder = internal.back();
    internal.pop_back();
  }
  stats.internal_count = internal.size();
  if (internal.empty()) {
    return stats;
  }
  double sum = 0.0;
  double abs_sum = 0.0;
  stats.min = internal.front();
  stats.max = internal.front();
  stats.max_abs_error = 0.0;
  for (std::size_t index = 0; index < internal.size(); ++index) {
    const double gap = internal[index];
    sum += gap;
    const double abs_error = std::abs(gap - kTargetSpacing);
    abs_sum += abs_error;
    stats.min = std::min(stats.min, gap);
    stats.max = std::max(stats.max, gap);
    if (abs_error >= stats.max_abs_error) {
      stats.max_abs_error = abs_error;
      stats.outlier_index = index;
      stats.outlier_gap = gap;
    }
  }
  stats.mean = sum / static_cast<double>(internal.size());
  stats.mean_abs_error = abs_sum / static_cast<double>(internal.size());
  double variance = 0.0;
  for (const double gap : internal) {
    const double delta = gap - stats.mean;
    variance += delta * delta;
  }
  stats.stddev = internal.size() > 1
                     ? std::sqrt(variance / static_cast<double>(internal.size() - 1))
                     : 0.0;
  return stats;
}

OrientationStats orientationOf(
    const std::vector<vision_path_generator::Waypoint>& waypoints,
    const vision_path_generator::ExtractResult& result) {
  OrientationStats stats;
  (void)result;
  if (waypoints.empty()) {
    return stats;
  }
  std::vector<double> errors;
  std::vector<double> adjacent;
  errors.reserve(waypoints.size());
  adjacent.reserve(waypoints.size());
  for (std::size_t index = 0; index < waypoints.size(); ++index) {
    const auto& waypoint = waypoints[index];
    const auto& quat = waypoint.orientation;
    const bool finite = std::isfinite(quat.x) && std::isfinite(quat.y) && std::isfinite(quat.z) &&
                        std::isfinite(quat.w);
    if (!finite) {
      ++stats.non_finite;
      ++stats.invalid_quaternion;
      continue;
    }
    const double norm = quatNorm(quat);
    if (norm < 1e-8) {
      ++stats.zero_quaternion;
      ++stats.invalid_quaternion;
      continue;
    }
    if (std::abs(norm - 1.0) > 1e-3) {
      ++stats.bad_norm;
      ++stats.invalid_quaternion;
    }
    if (index > 0 && quatDot(waypoints[index - 1].orientation, quat) < 0.0) {
      ++stats.quaternion_flips;
    }
    const double normal_norm = vision_path_generator::norm(waypoint.normal);
    if (normal_norm < 1e-8) {
      ++stats.hemisphere_violations;
      continue;
    }
    const vision_path_generator::Vec3 to_viewpoint{
        -waypoint.position.x, -waypoint.position.y, -waypoint.position.z};
    if (vision_path_generator::dot(waypoint.normal, to_viewpoint) < 0.0) {
      ++stats.hemisphere_violations;
    }
    if (index > 0) {
      const double adjacent_dot =
          vision_path_generator::dot(waypoints[index - 1].normal, waypoint.normal);
      adjacent.push_back(adjacent_dot);
      if (adjacent_dot < 0.0) {
        ++stats.normal_flips;
      }
    }
    const auto tool_z = vision_path_generator::rotateVector(quat, {0.0, 0.0, 1.0});
    const auto expected = waypoint.normal * -1.0;
    const double denom = vision_path_generator::norm(tool_z) * vision_path_generator::norm(expected);
    if (denom < 1e-12) {
      continue;
    }
    const double cosine = std::clamp(vision_path_generator::dot(tool_z, expected) / denom, -1.0, 1.0);
    const double degrees = std::acos(cosine) * (180.0 / kPi);
    errors.push_back(degrees);
  }
  if (!errors.empty()) {
    std::sort(errors.begin(), errors.end());
    double sum = 0.0;
    for (const double error : errors) {
      sum += error;
    }
    stats.mean_error_deg = sum / static_cast<double>(errors.size());
    stats.max_error_deg = errors.back();
    const auto p95_index = static_cast<std::size_t>(std::ceil(0.95 * static_cast<double>(errors.size()))) - 1;
    stats.p95_error_deg = errors[std::min(p95_index, errors.size() - 1)];
    stats.recorded = true;
  }
  if (!adjacent.empty()) {
    double sum = 0.0;
    for (const double value : adjacent) {
      sum += value;
    }
    stats.mean_adjacent_dot = sum / static_cast<double>(adjacent.size());
    std::sort(adjacent.begin(), adjacent.end());
    stats.min_adjacent_dot = adjacent.front();
    const double p05_pos = 0.05 * static_cast<double>(adjacent.size() - 1);
    const auto p05_index = static_cast<std::size_t>(std::floor(p05_pos));
    const double p05_frac = p05_pos - static_cast<double>(p05_index);
    const double upper = adjacent[std::min(p05_index + 1, adjacent.size() - 1)];
    stats.p05_adjacent_dot = adjacent[p05_index] * (1.0 - p05_frac) + upper * p05_frac;
    const double cosine = std::clamp(stats.min_adjacent_dot, -1.0, 1.0);
    stats.max_adjacent_angle_deg = std::acos(cosine) * (180.0 / kPi);
  }
  return stats;
}

void writeCsv(const std::string& path, const std::vector<vision_path_generator::Waypoint>& waypoints) {
  std::ofstream output(path);
  output << "index,x,y,z,qx,qy,qz,qw,nx,ny,nz,path_distance_m\n";
  output << std::setprecision(17);
  double distance = 0.0;
  for (std::size_t index = 0; index < waypoints.size(); ++index) {
    if (index > 0) {
      distance += vision_path_generator::norm(waypoints[index].position - waypoints[index - 1].position);
    }
    const auto& waypoint = waypoints[index];
    output << index << ',' << waypoint.position.x << ',' << waypoint.position.y << ',' << waypoint.position.z
           << ',' << waypoint.orientation.x << ',' << waypoint.orientation.y << ',' << waypoint.orientation.z
           << ',' << waypoint.orientation.w << ',' << waypoint.normal.x << ',' << waypoint.normal.y << ','
           << waypoint.normal.z << ',' << distance << '\n';
  }
}

void writeSvg(
    const std::string& path,
    const pcl::PointCloud<pcl::PointXYZ>& object,
    const std::vector<vision_path_generator::Waypoint>& waypoints) {
  std::vector<vision_path_generator::Vec3> points;
  const std::size_t stride = std::max<std::size_t>(1, object.size() / 4000);
  for (std::size_t index = 0; index < object.size(); index += stride) {
    points.push_back({object[index].x, object[index].y, object[index].z});
  }
  for (const auto& waypoint : waypoints) {
    points.push_back(waypoint.position);
  }
  if (points.empty()) {
    std::ofstream output(path);
    output << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"640\" height=\"480\"></svg>\n";
    return;
  }
  double min_x = points.front().x;
  double max_x = points.front().x;
  double min_y = points.front().y;
  double max_y = points.front().y;
  for (const auto& point : points) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }
  const double span_x = std::max(1e-6, max_x - min_x);
  const double span_y = std::max(1e-6, max_y - min_y);
  const double scale = 600.0 / std::max(span_x, span_y);
  auto map_x = [&](double x) { return 20.0 + ((x - min_x) * scale); };
  auto map_y = [&](double y) { return 20.0 + ((y - min_y) * scale); };
  std::ofstream output(path);
  output << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"680\" height=\"680\">\n";
  output << "<rect width=\"680\" height=\"680\" fill=\"white\"/>\n";
  for (std::size_t index = 0; index < object.size(); index += stride) {
    output << "<circle cx=\"" << map_x(object[index].x) << "\" cy=\"" << map_y(object[index].y)
           << "\" r=\"1.2\" fill=\"#9aa0a6\"/>\n";
  }
  if (waypoints.size() >= 2) {
    output << "<polyline fill=\"none\" stroke=\"#1a73e8\" stroke-width=\"1.5\" points=\"";
    for (const auto& waypoint : waypoints) {
      output << map_x(waypoint.position.x) << ',' << map_y(waypoint.position.y) << ' ';
    }
    output << "\"/>\n";
  }
  for (const auto& waypoint : waypoints) {
    const double x = map_x(waypoint.position.x);
    const double y = map_y(waypoint.position.y);
    output << "<circle cx=\"" << x << "\" cy=\"" << y << "\" r=\"2.2\" fill=\"#d93025\"/>\n";
    const double arrow = 18.0;
    output << "<line x1=\"" << x << "\" y1=\"" << y << "\" x2=\"" << (x + (waypoint.normal.x * arrow))
           << "\" y2=\"" << (y + (waypoint.normal.y * arrow))
           << "\" stroke=\"#188038\" stroke-width=\"1\"/>\n";
  }
  output << "</svg>\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "사용법: geometry_report <snapshot.pcd> <meta.txt> <output_dir>\n";
    return 2;
  }
  const std::string pcd_path = argv[1];
  const std::string meta_path = argv[2];
  const std::string output_dir = argv[3];
  const auto meta = readMeta(meta_path);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile(pcd_path, *cloud) < 0 || cloud->empty()) {
    std::cerr << "스냅샷 PCD를 읽지 못했습니다.\n";
    return 1;
  }

  vision_path_generator::ExtractParams params;
  params.path_mode = "boundary";
  vision_path_generator::PathExtractor extractor(params);

  std::vector<vision_path_generator::ExtractResult> runs;
  runs.reserve(3);
  for (int run = 0; run < 3; ++run) {
    runs.push_back(extractor.extract(cloud));
  }
  const auto& primary = runs.front();
  const auto spacing = spacingOf(primary.waypoints);
  const auto orientation = orientationOf(primary.waypoints, primary);
  const auto orientation_run2 = orientationOf(runs[1].waypoints, runs[1]);
  const auto orientation_run3 = orientationOf(runs[2].waypoints, runs[2]);
  const bool deterministic = sameRun(runs[0], runs[1]) && sameRun(runs[1], runs[2]);
  const bool spacing_measured = spacing.internal_count > 0;
  const bool spacing_pass =
      spacing_measured && std::abs(spacing.mean - kTargetSpacing) <= kSpacingTolerance;
  const bool geometry_pass = primary.error.empty() && primary.method == "boundary" &&
                             primary.finite_points > 0 && primary.voxel_points > 0 &&
                             primary.sor_points > 0 && primary.plane_valid && primary.plane_inliers > 0 &&
                             primary.object_points > 0 && primary.surface_nan == 0 &&
                             primary.surface_inf == 0 && primary.finite_normals > 0 &&
                             primary.invalid_normals == 0 && orientation.normal_flips == 0 &&
                             orientation_run2.normal_flips == 0 && orientation_run3.normal_flips == 0 &&
                             orientation.hemisphere_violations == 0 &&
                             orientation_run2.hemisphere_violations == 0 &&
                             orientation_run3.hemisphere_violations == 0 &&
                             primary.raw_path_length > 0.0 && primary.waypoints.size() > 1 &&
                             spacing_pass && orientation.invalid_quaternion == 0 &&
                             orientation.quaternion_flips == 0 && orientation.recorded && deterministic;

  writeCsv(output_dir + "/waypoints.csv", primary.waypoints);
  writeCsv(output_dir + "/waypoints-run2.csv", runs[1].waypoints);
  writeCsv(output_dir + "/waypoints-run3.csv", runs[2].waypoints);
  writeSvg(output_dir + "/path.svg", primary.object ? *primary.object : pcl::PointCloud<pcl::PointXYZ>(), primary.waypoints);

  std::ofstream parameters(output_dir + "/parameters.yaml");
  parameters << "path_mode: boundary\n";
  parameters << "voxel_leaf_size: " << params.voxel_leaf_size << "\n";
  parameters << "sor_mean_k: " << params.sor_mean_k << "\n";
  parameters << "sor_stddev: " << params.sor_stddev << "\n";
  parameters << "ransac_distance: " << params.ransac_distance << "\n";
  parameters << "ransac_iterations: " << params.ransac_iterations << "\n";
  parameters << "ransac_seed: pcl_1_12_default_12345\n";
  parameters << "max_plane_inlier_ratio: " << params.max_plane_inlier_ratio << "\n";
  parameters << "min_height: " << params.min_height << "\n";
  parameters << "normal_k: " << params.normal_k << "\n";
  parameters << "normal_viewpoint: [0.0, 0.0, 0.0]\n";
  parameters << "normal_orientation: viewpoint\n";
  parameters << "boundary_radius: " << params.boundary_radius << "\n";
  parameters << "waypoint_spacing: " << params.waypoint_spacing << "\n";
  parameters << "max_segment_gap: " << params.max_segment_gap << "\n";
  parameters << "smooth_passes: " << params.smooth_passes << "\n";
  parameters << "cluster_tolerance: " << params.cluster_tolerance << "\n";
  parameters << "min_cluster_size: " << params.min_cluster_size << "\n";
  parameters << "min_waypoints: " << params.min_waypoints << "\n";
  parameters << "target_frame: camera_depth_optical_frame\n";
  parameters << "require_transform: false\n";

  auto meta_or = [&](const std::string& key) {
    const auto found = meta.find(key);
    return found == meta.end() ? std::string() : found->second;
  };

  std::ofstream json(output_dir + "/geometry-results.json");
  json << std::setprecision(17);
  json << "{\n";
  json << "  \"status\": \"" << (geometry_pass ? "PASS" : "FAIL") << "\",\n";
  json << "  \"frame\": \"" << jsonEscape(meta_or("frame_id")) << "\",\n";
  json << "  \"error\": \"" << jsonEscape(primary.error) << "\",\n";
  json << "  \"input\": {\n";
  json << "    \"topic\": \"/camera/depth/points\",\n";
  json << "    \"frame_id\": \"" << jsonEscape(meta_or("frame_id")) << "\",\n";
  json << "    \"stamp_sec\": " << (meta_or("stamp_sec").empty() ? "null" : meta_or("stamp_sec")) << ",\n";
  json << "    \"stamp_nanosec\": " << (meta_or("stamp_nanosec").empty() ? "null" : meta_or("stamp_nanosec"))
       << ",\n";
  json << "    \"width\": " << (meta_or("width").empty() ? "null" : meta_or("width")) << ",\n";
  json << "    \"height\": " << (meta_or("height").empty() ? "null" : meta_or("height")) << ",\n";
  json << "    \"fields\": \"" << jsonEscape(meta_or("fields")) << "\",\n";
  json << "    \"sha256\": \"" << jsonEscape(meta_or("sha256")) << "\",\n";
  json << "    \"loaded_points\": " << cloud->size() << ",\n";
  json << "    \"finite_points\": " << primary.finite_points << "\n";
  json << "  },\n";
  json << "  \"filtering\": {\n";
  json << "    \"input\": " << primary.input_points << ",\n";
  json << "    \"finite\": " << primary.finite_points << ",\n";
  json << "    \"after_voxel\": " << primary.voxel_points << ",\n";
  json << "    \"voxel_index_fallback\": " << (primary.voxel_index_fallback ? "true" : "false") << ",\n";
  json << "    \"after_outlier\": " << primary.sor_points << ",\n";
  json << "    \"plane_inliers\": " << primary.plane_inliers << ",\n";
  json << "    \"after_plane_removal\": " << primary.plane_remaining << ",\n";
  json << "    \"after_height\": " << primary.height_points << ",\n";
  json << "    \"surface_points\": " << primary.object_points << "\n";
  json << "  },\n";
  json << "  \"plane\": {\n";
  json << "    \"valid\": " << (primary.plane_valid ? "true" : "false") << ",\n";
  json << "    \"oriented_toward_object\": " << (primary.plane_oriented ? "true" : "false") << ",\n";
  json << "    \"a\": " << jsonNumber(primary.plane_a) << ",\n";
  json << "    \"b\": " << jsonNumber(primary.plane_b) << ",\n";
  json << "    \"c\": " << jsonNumber(primary.plane_c) << ",\n";
  json << "    \"d\": " << jsonNumber(primary.plane_d) << ",\n";
  json << "    \"inliers\": " << primary.plane_inliers << ",\n";
  json << "    \"inlier_ratio\": " << jsonNumber(primary.plane_inlier_ratio) << ",\n";
  json << "    \"remaining\": " << primary.plane_remaining << "\n";
  json << "  },\n";
  json << "  \"normals\": {\n";
  json << "    \"count\": " << primary.normal_count << ",\n";
  json << "    \"finite\": " << primary.finite_normals << ",\n";
  json << "    \"invalid\": " << primary.invalid_normals << ",\n";
  json << "    \"length_mean\": " << jsonNumber(primary.normal_length_mean) << ",\n";
  json << "    \"length_min\": " << jsonNumber(primary.normal_length_min) << ",\n";
  json << "    \"length_max\": " << jsonNumber(primary.normal_length_max) << ",\n";
  json << "    \"surface_nan_coordinates\": " << primary.surface_nan << ",\n";
  json << "    \"surface_inf_coordinates\": " << primary.surface_inf << ",\n";
  json << "    \"consecutive_negative_dots\": " << orientation.normal_flips << ",\n";
  json << "    \"hemisphere_violations\": " << orientation.hemisphere_violations << ",\n";
  json << "    \"viewpoint\": [0.0, 0.0, 0.0],\n";
  json << "    \"hemisphere_rule\": \"n · (viewpoint - point) >= 0\",\n";
  json << "    \"min_adjacent_dot\": " << jsonNumber(orientation.min_adjacent_dot) << ",\n";
  json << "    \"mean_adjacent_dot\": " << jsonNumber(orientation.mean_adjacent_dot) << ",\n";
  json << "    \"p05_adjacent_dot\": " << jsonNumber(orientation.p05_adjacent_dot) << ",\n";
  json << "    \"max_adjacent_angle_deg\": " << jsonNumber(orientation.max_adjacent_angle_deg) << "\n";
  json << "  },\n";
  json << "  \"path\": {\n";
  json << "    \"mode\": \"" << jsonEscape(primary.method) << "\",\n";
  json << "    \"requested_mode\": \"boundary\",\n";
  json << "    \"raw_points\": " << primary.raw_path_points << ",\n";
  json << "    \"length_m\": " << jsonNumber(primary.raw_path_length) << ",\n";
  json << "    \"closed\": false,\n";
  json << "    \"endpoint_gap_m\": " << jsonNumber(primary.endpoint_gap) << "\n";
  json << "  },\n";
  json << "  \"spacing\": {\n";
  json << "    \"target_spacing_m\": " << kTargetSpacing << ",\n";
  json << "    \"internal_segment_count\": " << spacing.internal_count << ",\n";
  json << "    \"mean_spacing_m\": " << jsonNumber(spacing.mean) << ",\n";
  json << "    \"std_spacing_m\": " << jsonNumber(spacing.stddev) << ",\n";
  json << "    \"min_spacing_m\": " << jsonNumber(spacing.min) << ",\n";
  json << "    \"max_spacing_m\": " << jsonNumber(spacing.max) << ",\n";
  json << "    \"mean_abs_error_m\": " << jsonNumber(spacing.mean_abs_error) << ",\n";
  json << "    \"max_abs_error_m\": " << jsonNumber(spacing.max_abs_error) << ",\n";
  json << "    \"remainder_m\": " << jsonNumber(spacing.remainder) << ",\n";
  json << "    \"mean_error_within_0_25mm\": " << (spacing_pass ? "true" : "false") << "\n";
  json << "  },\n";
  json << "  \"orientation\": {\n";
  json << "    \"tool_axis\": \"+Z\",\n";
  json << "    \"approach\": \"tool +Z equals the negated outward surface normal\",\n";
  json << "    \"roll\": \"path tangent projected onto the plane perpendicular to tool +Z\",\n";
  json << "    \"invalid_quaternion\": " << orientation.invalid_quaternion << ",\n";
  json << "    \"zero_quaternion\": " << orientation.zero_quaternion << ",\n";
  json << "    \"bad_norm\": " << orientation.bad_norm << ",\n";
  json << "    \"non_finite\": " << orientation.non_finite << ",\n";
  json << "    \"quaternion_sign_flips\": " << orientation.quaternion_flips << ",\n";
  json << "    \"mean_error_deg\": " << jsonNumber(orientation.mean_error_deg) << ",\n";
  json << "    \"p95_error_deg\": " << jsonNumber(orientation.p95_error_deg) << ",\n";
  json << "    \"max_error_deg\": " << jsonNumber(orientation.max_error_deg) << ",\n";
  json << "    \"recorded\": " << (orientation.recorded ? "true" : "false") << "\n";
  json << "  },\n";
  json << "  \"determinism\": {\n";
  json << "    \"runs\": 3,\n";
  json << "    \"equal\": " << (deterministic ? "true" : "false") << ",\n";
  json << "    \"waypoint_counts\": [" << runs[0].waypoints.size() << ", " << runs[1].waypoints.size()
       << ", " << runs[2].waypoints.size() << "],\n";
  json << "    \"path_lengths_m\": [" << jsonNumber(runs[0].raw_path_length) << ", "
       << jsonNumber(runs[1].raw_path_length) << ", " << jsonNumber(runs[2].raw_path_length) << "],\n";
  json << "    \"fingerprints_equal\": " << (deterministic ? "true" : "false") << ",\n";
  json << "    \"normal_flips\": [" << orientation.normal_flips << ", " << orientation_run2.normal_flips
       << ", " << orientation_run3.normal_flips << "]\n";
  json << "  },\n";
  json << "  \"timing_ms\": {\n";
  json << "    \"voxel_ms\": " << jsonNumber(primary.voxel_ms) << ",\n";
  json << "    \"outlier_ms\": " << jsonNumber(primary.outlier_ms) << ",\n";
  json << "    \"plane_ms\": " << jsonNumber(primary.plane_ms) << ",\n";
  json << "    \"normal_ms\": " << jsonNumber(primary.normal_ms) << ",\n";
  json << "    \"path_ms\": " << jsonNumber(primary.path_ms) << ",\n";
  json << "    \"waypoint_ms\": " << jsonNumber(primary.waypoint_ms) << ",\n";
  json << "    \"total_ms\": " << jsonNumber(primary.total_ms) << ",\n";
  json << "    \"run_totals_ms\": [" << jsonNumber(runs[0].total_ms) << ", " << jsonNumber(runs[1].total_ms)
       << ", " << jsonNumber(runs[2].total_ms) << "]\n";
  json << "  }\n";
  json << "}\n";

  auto write_text = [&](const std::string& name, const std::string& body) {
    std::ofstream output(output_dir + "/" + name);
    output << body;
  };
  write_text(
      "filtering.txt",
      "INPUT " + std::to_string(primary.input_points) + "\nAFTER_VOXEL " +
          std::to_string(primary.voxel_points) + "\nVOXEL_INDEX_FALLBACK " +
          std::string(primary.voxel_index_fallback ? "true" : "false") + "\nAFTER_OUTLIER_REMOVAL " +
          std::to_string(primary.sor_points) + "\nAFTER_PLANE_REMOVAL " +
          std::to_string(primary.plane_remaining) + "\nAFTER_HEIGHT " +
          std::to_string(primary.height_points) + "\nPATH_CANDIDATES " +
          std::to_string(primary.raw_path_points) + "\nWAYPOINTS " +
          std::to_string(primary.waypoints.size()) + "\n");
  {
    std::ostringstream body;
    body << std::setprecision(17);
    body << "valid " << (primary.plane_valid ? "true" : "false") << "\n";
    body << "oriented_toward_object " << (primary.plane_oriented ? "true" : "false") << "\n";
    body << "coefficients " << primary.plane_a << " " << primary.plane_b << " " << primary.plane_c << " "
         << primary.plane_d << "\n";
    body << "normal " << primary.plane_a << " " << primary.plane_b << " " << primary.plane_c << "\n";
    body << "inliers " << primary.plane_inliers << "\n";
    body << "inlier_ratio " << primary.plane_inlier_ratio << "\n";
    body << "remaining " << primary.plane_remaining << "\n";
    write_text("plane.txt", body.str());
  }
  {
    std::ostringstream body;
    body << std::setprecision(17);
    body << "normal_count " << primary.normal_count << "\n";
    body << "finite_normals " << primary.finite_normals << "\n";
    body << "invalid_normals " << primary.invalid_normals << "\n";
    body << "normal_length_mean " << primary.normal_length_mean << "\n";
    body << "normal_length_min " << primary.normal_length_min << "\n";
    body << "normal_length_max " << primary.normal_length_max << "\n";
    body << "surface_nan " << primary.surface_nan << "\n";
    body << "surface_inf " << primary.surface_inf << "\n";
    body << "unexpected_normal_flips " << orientation.normal_flips << "\n";
    body << "hemisphere_violations " << orientation.hemisphere_violations << "\n";
    body << "min_adjacent_dot " << orientation.min_adjacent_dot << "\n";
    body << "mean_adjacent_dot " << orientation.mean_adjacent_dot << "\n";
    body << "p05_adjacent_dot " << orientation.p05_adjacent_dot << "\n";
    body << "max_adjacent_angle_deg " << orientation.max_adjacent_angle_deg << "\n";
    write_text("normals.txt", body.str());
    write_text("normals-after.txt", body.str());
  }
  {
    std::ostringstream body;
    body << std::setprecision(17);
    body << "mode " << primary.method << "\n";
    body << "requested_mode boundary\n";
    body << "raw_path_points " << primary.raw_path_points << "\n";
    body << "path_length_m " << primary.raw_path_length << "\n";
    body << "path_closed no\n";
    body << "endpoint_gap_m " << primary.endpoint_gap << "\n";
    body << "error " << primary.error << "\n";
    write_text("path.txt", body.str());
  }
  {
    std::ostringstream body;
    body << std::setprecision(17);
    body << "target_spacing_m " << kTargetSpacing << "\n";
    body << "internal_segment_count " << spacing.internal_count << "\n";
    body << "mean_spacing_m " << spacing.mean << "\n";
    body << "std_spacing_m " << spacing.stddev << "\n";
    body << "min_spacing_m " << spacing.min << "\n";
    body << "max_spacing_m " << spacing.max << "\n";
    body << "mean_abs_error_m " << spacing.mean_abs_error << "\n";
    body << "max_abs_error_m " << spacing.max_abs_error << "\n";
    body << "remainder_m " << spacing.remainder << "\n";
    body << "mean_error_within_0_25mm " << (spacing_pass ? "true" : "false") << "\n";
    write_text("spacing.txt", body.str());
  }
  {
    std::ostringstream body;
    body << std::setprecision(17);
    body << "tool_axis +Z\n";
    body << "approach tool +Z follows the negated outward surface normal\n";
    body << "roll path tangent projected orthogonal to tool +Z\n";
    body << "invalid_quaternion " << orientation.invalid_quaternion << "\n";
    body << "quaternion_sign_flips " << orientation.quaternion_flips << "\n";
    body << "mean_error_deg " << orientation.mean_error_deg << "\n";
    body << "p95_error_deg " << orientation.p95_error_deg << "\n";
    body << "max_error_deg " << orientation.max_error_deg << "\n";
    body << "hemisphere_violations " << orientation.hemisphere_violations << "\n";
    write_text("orientation.txt", body.str());
    write_text("orientation-after.txt", body.str());
  }
  {
    std::ostringstream body;
    body << std::setprecision(17);
    body << "index,dot,angle_deg,camera_facing_i,camera_facing_next\n";
    for (std::size_t index = 1; index < primary.waypoints.size(); ++index) {
      const auto& previous = primary.waypoints[index - 1];
      const auto& current = primary.waypoints[index];
      const double adjacent_dot = vision_path_generator::dot(previous.normal, current.normal);
      const double angle = std::acos(std::clamp(adjacent_dot, -1.0, 1.0)) * (180.0 / kPi);
      const auto facing = [](const vision_path_generator::Waypoint& waypoint) {
        const vision_path_generator::Vec3 to_viewpoint{
            -waypoint.position.x, -waypoint.position.y, -waypoint.position.z};
        return vision_path_generator::dot(waypoint.normal, to_viewpoint) >= 0.0 ? 1 : -1;
      };
      body << (index - 1) << ',' << adjacent_dot << ',' << angle << ',' << facing(previous) << ','
           << facing(current) << '\n';
    }
    write_text("normal-continuity-after.csv", body.str());
  }
  {
    std::ostringstream body;
    body << std::setprecision(17);
    body << "outlier_index " << spacing.outlier_index << "\n";
    body << "gap_m " << spacing.outlier_gap << "\n";
    body << "abs_error_m " << spacing.max_abs_error << "\n";
    const auto point = [](const vision_path_generator::Vec3& value) {
      std::ostringstream text;
      text << std::setprecision(17) << value.x << " " << value.y << " " << value.z;
      return text.str();
    };
    if (spacing.outlier_index + 1 < primary.waypoints.size()) {
      const std::size_t index = spacing.outlier_index;
      body << "previous " << (index > 0 ? point(primary.waypoints[index - 1].position) : "none") << "\n";
      body << "current " << point(primary.waypoints[index].position) << "\n";
      body << "next " << point(primary.waypoints[index + 1].position) << "\n";
    }
    write_text("spacing-outlier.txt", body.str());
  }
  write_text(
      "normal-orientation-rule.txt",
      "viewpoint 0 0 0\n"
      "frame camera_depth_optical_frame\n"
      "hemisphere_rule n · (viewpoint - point) >= 0\n"
      "outward camera-facing side of the visible surface\n"
      "tool_axis +Z = -outward normal\n"
      "roll path tangent projected orthogonal to tool +Z\n"
      "plane_normal_sign_alignment not used\n"
      "neighbor_sign_propagation not used\n");
  {
    std::ostringstream body;
    body << std::setprecision(17);
    body << "runs 3\n";
    body << "equal " << (deterministic ? "true" : "false") << "\n";
    body << "waypoint_counts " << runs[0].waypoints.size() << " " << runs[1].waypoints.size() << " "
         << runs[2].waypoints.size() << "\n";
    body << "path_lengths_m " << runs[0].raw_path_length << " " << runs[1].raw_path_length << " "
         << runs[2].raw_path_length << "\n";
    body << "normal_flips " << orientation.normal_flips << " " << orientation_run2.normal_flips << " "
         << orientation_run3.normal_flips << "\n";
    body << "hemisphere_violations " << orientation.hemisphere_violations << " "
         << orientation_run2.hemisphere_violations << " " << orientation_run3.hemisphere_violations << "\n";
    write_text("determinism.txt", body.str());
  }
  {
    std::ostringstream body;
    body << std::setprecision(17);
    body << "voxel_ms " << primary.voxel_ms << "\n";
    body << "outlier_ms " << primary.outlier_ms << "\n";
    body << "plane_ms " << primary.plane_ms << "\n";
    body << "normal_ms " << primary.normal_ms << "\n";
    body << "path_ms " << primary.path_ms << "\n";
    body << "waypoint_ms " << primary.waypoint_ms << "\n";
    body << "total_ms " << primary.total_ms << "\n";
    write_text("timing.txt", body.str());
  }

  std::cout << "P-004 GEOMETRY " << (geometry_pass ? "PASS" : "FAIL") << "\n";
  std::cout << "waypoints " << primary.waypoints.size() << " method " << primary.method << " error "
            << primary.error << "\n";
  return geometry_pass ? 0 : 3;
}
