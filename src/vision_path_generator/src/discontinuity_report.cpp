// Copyright 2026 cad-less-robotics contributors
// 진단 전용이다. PathExtractor의 법선·경로 계산은 호출만 하고 바꾸지 않는다.

#include "vision_path_generator/path_extractor.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kVizRadius = 0.02;
constexpr int kWindow = 10;

double dot3(const vision_path_generator::Vec3& a, const vision_path_generator::Vec3& b) {
  return vision_path_generator::dot(a, b);
}

vision_path_generator::Vec3 unit(const vision_path_generator::Vec3& value) {
  const double length = vision_path_generator::norm(value);
  if (length < 1e-12) {
    return {};
  }
  return value * (1.0 / length);
}

double angleDeg(double cosine) {
  const double clamped = std::clamp(cosine, -1.0, 1.0);
  return std::acos(clamped) * (180.0 / kPi);
}

vision_path_generator::Vec3 tangentAt(
    const std::vector<vision_path_generator::Waypoint>& waypoints, std::size_t index) {
  if (waypoints.size() < 2) {
    return {};
  }
  if (index + 1 < waypoints.size()) {
    return unit(waypoints[index + 1].position - waypoints[index].position);
  }
  return unit(waypoints[index].position - waypoints[index - 1].position);
}

struct Pca {
  int count{0};
  vision_path_generator::Vec3 centroid;
  double eigenvalue_0{0.0};
  double eigenvalue_1{0.0};
  double eigenvalue_2{0.0};
  vision_path_generator::Vec3 normal;
  double camera_dot{0.0};
};

Pca analyzeNeighbors(
    const std::vector<vision_path_generator::Vec3>& points,
    const vision_path_generator::Vec3& orient_at) {
  Pca result;
  result.count = static_cast<int>(points.size());
  if (points.empty()) {
    return result;
  }
  for (const auto& point : points) {
    result.centroid = result.centroid + point;
  }
  result.centroid = result.centroid * (1.0 / static_cast<double>(points.size()));
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto& point : points) {
    const Eigen::Vector3d delta(
        point.x - result.centroid.x, point.y - result.centroid.y, point.z - result.centroid.z);
    covariance += delta * delta.transpose();
  }
  covariance /= static_cast<double>(points.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  result.eigenvalue_0 = solver.eigenvalues()(0);
  result.eigenvalue_1 = solver.eigenvalues()(1);
  result.eigenvalue_2 = solver.eigenvalues()(2);
  const Eigen::Vector3d axis = solver.eigenvectors().col(0);
  result.normal = vision_path_generator::orientNormalTowardsViewpoint(
      orient_at, {axis.x(), axis.y(), axis.z()}, {0.0, 0.0, 0.0});
  const vision_path_generator::Vec3 to_camera{-orient_at.x, -orient_at.y, -orient_at.z};
  result.camera_dot = dot3(result.normal, to_camera);
  return result;
}

std::vector<vision_path_generator::Vec3> nearestPoints(
    const pcl::KdTreeFLANN<pcl::PointXYZ>& tree,
    const vision_path_generator::Vec3& query,
    int k,
    std::vector<float>* distances) {
  pcl::PointXYZ point;
  point.x = static_cast<float>(query.x);
  point.y = static_cast<float>(query.y);
  point.z = static_cast<float>(query.z);
  std::vector<int> ids(static_cast<std::size_t>(k));
  std::vector<float> squared(static_cast<std::size_t>(k));
  const int found = tree.nearestKSearch(point, k, ids, squared);
  std::vector<vision_path_generator::Vec3> output;
  output.reserve(static_cast<std::size_t>(std::max(found, 0)));
  if (distances != nullptr) {
    distances->clear();
  }
  for (int index = 0; index < found; ++index) {
    const auto& source = tree.getInputCloud()->at(static_cast<std::size_t>(ids[static_cast<std::size_t>(index)]));
    output.push_back({source.x, source.y, source.z});
    if (distances != nullptr) {
      distances->push_back(std::sqrt(squared[static_cast<std::size_t>(index)]));
    }
  }
  return output;
}

std::vector<vision_path_generator::Vec3> radiusPoints(
    const pcl::KdTreeFLANN<pcl::PointXYZ>& tree, const vision_path_generator::Vec3& query, double radius) {
  pcl::PointXYZ point;
  point.x = static_cast<float>(query.x);
  point.y = static_cast<float>(query.y);
  point.z = static_cast<float>(query.z);
  std::vector<int> ids;
  std::vector<float> squared;
  tree.radiusSearch(point, radius, ids, squared);
  std::vector<vision_path_generator::Vec3> output;
  output.reserve(ids.size());
  for (const int id : ids) {
    const auto& source = tree.getInputCloud()->at(static_cast<std::size_t>(id));
    output.push_back({source.x, source.y, source.z});
  }
  return output;
}

void writeSurface(
    const std::string& path,
    const std::vector<vision_path_generator::Vec3>& normal_k,
    const std::vector<float>& normal_k_distance,
    const std::vector<vision_path_generator::Vec3>& viz) {
  std::ofstream output(path);
  output << std::setprecision(17);
  output << "set,rank,x,y,z,distance_m\n";
  for (std::size_t index = 0; index < normal_k.size(); ++index) {
    output << "normal_k," << index << ',' << normal_k[index].x << ',' << normal_k[index].y << ','
           << normal_k[index].z << ',' << normal_k_distance[index] << '\n';
  }
  for (std::size_t index = 0; index < viz.size(); ++index) {
    output << "viz_radius_0_02," << index << ',' << viz[index].x << ',' << viz[index].y << ','
           << viz[index].z << ",\n";
  }
}

std::string indexName(int index) {
  std::ostringstream name;
  name << std::setw(3) << std::setfill('0') << index;
  return name.str();
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "사용법: discontinuity_report <snapshot.pcd> <output_dir> [index ...]\n";
    return 2;
  }
  const std::string pcd_path = argv[1];
  const std::string output_dir = argv[2];
  std::vector<int> problems;
  for (int arg = 3; arg < argc; ++arg) {
    problems.push_back(std::stoi(argv[arg]));
  }
  if (problems.empty()) {
    problems = {43, 44, 311};
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile(pcd_path, *cloud) < 0 || cloud->empty()) {
    std::cerr << "스냅샷 PCD를 읽지 못했습니다.\n";
    return 1;
  }

  vision_path_generator::ExtractParams params;
  params.path_mode = "boundary";
  const vision_path_generator::ExtractResult result =
      vision_path_generator::PathExtractor(params).extract(cloud);
  if (!result.error.empty() || !result.object || result.waypoints.size() < 2) {
    std::cerr << "기존 경로 추출이 실패했습니다: " << result.error << "\n";
    return 1;
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(result.object);
  const auto& waypoints = result.waypoints;

  std::ofstream discontinuities(output_dir + "/discontinuities.csv");
  std::ofstream tangents(output_dir + "/tangent-analysis.csv");
  std::ofstream geometry(output_dir + "/local-geometry-analysis.csv");
  std::ofstream signs(output_dir + "/alternative-sign-analysis.csv");
  discontinuities << std::setprecision(17);
  tangents << std::setprecision(17);
  geometry << std::setprecision(17);
  signs << std::setprecision(17);
  discontinuities << "index,next_index,dot,angle_deg,distance_m,tangent_angle_deg,"
                     "camera_dot_i,camera_dot_next,camera_cosine_i,camera_cosine_next\n";
  tangents << "index,tx,ty,tz,angle_to_next_tangent_deg,distance_to_next_m\n";
  geometry << "index,nearest_distance_m,normal_k_count,centroid_x,centroid_y,centroid_z,"
              "eigenvalue_0,eigenvalue_1,eigenvalue_2,surface_variation,planarity,"
              "pca_nx,pca_ny,pca_nz,pca_camera_dot,angle_pca_vs_waypoint_deg,"
              "source_neighbor_count,source_eigenvalue_0,source_eigenvalue_1,source_eigenvalue_2,"
              "source_surface_variation,angle_pca_source_vs_waypoint_deg,viz_point_count\n";
  signs << "index,flipped_waypoint,camera_dot,previous_dot,pair_dot,next_dot\n";

  std::vector<char> tangent_written(waypoints.size(), 0);
  auto write_tangent = [&](std::size_t index) {
    if (index >= waypoints.size() || tangent_written[index] != 0) {
      return;
    }
    tangent_written[index] = 1;
    const auto leaving = tangentAt(waypoints, index);
    double turn = 0.0;
    double distance = 0.0;
    if (index + 1 < waypoints.size()) {
      distance = vision_path_generator::norm(waypoints[index + 1].position - waypoints[index].position);
      if (index + 2 < waypoints.size()) {
        const auto next = tangentAt(waypoints, index + 1);
        const double denom = vision_path_generator::norm(leaving) * vision_path_generator::norm(next);
        turn = denom < 1e-12 ? 0.0 : angleDeg(dot3(leaving, next) / denom);
      }
    }
    tangents << index << ',' << leaving.x << ',' << leaving.y << ',' << leaving.z << ',' << turn << ','
             << distance << '\n';
  };

  for (const int problem : problems) {
    if (problem < 0 || static_cast<std::size_t>(problem + 1) >= waypoints.size()) {
      std::cerr << "인덱스 범위 밖: " << problem << "\n";
      return 1;
    }
    const auto index = static_cast<std::size_t>(problem);
    const int begin = std::max(0, problem - kWindow);
    const int end = std::min(static_cast<int>(waypoints.size()) - 1, problem + kWindow);

    std::ofstream neighborhood(output_dir + "/index-" + indexName(problem) + "-neighborhood.csv");
    neighborhood << std::setprecision(17);
    neighborhood << "index,x,y,z,nx,ny,nz,camera_facing_dot,previous_normal_dot,next_normal_dot,"
                    "tx,ty,tz,distance_from_previous,distance_from_next\n";
    for (int cursor = begin; cursor <= end; ++cursor) {
      const auto at = static_cast<std::size_t>(cursor);
      const auto& waypoint = waypoints[at];
      const auto to_camera = vision_path_generator::Vec3{-waypoint.position.x, -waypoint.position.y, -waypoint.position.z};
      const auto leaving = tangentAt(waypoints, at);
      const double previous_dot = at == 0 ? 0.0 : dot3(waypoints[at - 1].normal, waypoint.normal);
      const double next_dot =
          at + 1 < waypoints.size() ? dot3(waypoint.normal, waypoints[at + 1].normal) : 0.0;
      const double previous_distance =
          at == 0 ? 0.0 : vision_path_generator::norm(waypoint.position - waypoints[at - 1].position);
      const double next_distance =
          at + 1 < waypoints.size()
              ? vision_path_generator::norm(waypoints[at + 1].position - waypoint.position)
              : 0.0;
      neighborhood << cursor << ',' << waypoint.position.x << ',' << waypoint.position.y << ','
                   << waypoint.position.z << ',' << waypoint.normal.x << ',' << waypoint.normal.y << ','
                   << waypoint.normal.z << ',' << dot3(waypoint.normal, to_camera) << ',';
      if (at == 0) {
        neighborhood << ',';
      } else {
        neighborhood << previous_dot << ',';
      }
      if (at + 1 >= waypoints.size()) {
        neighborhood << ',';
      } else {
        neighborhood << next_dot << ',';
      }
      neighborhood << leaving.x << ',' << leaving.y << ',' << leaving.z << ',' << previous_distance << ','
                   << next_distance << '\n';
      write_tangent(at);
    }

    const auto& current = waypoints[index];
    const auto& next = waypoints[index + 1];
    const double pair_dot = dot3(current.normal, next.normal);
    const auto current_camera = vision_path_generator::Vec3{-current.position.x, -current.position.y, -current.position.z};
    const auto next_camera = vision_path_generator::Vec3{-next.position.x, -next.position.y, -next.position.z};
    const double current_camera_dot = dot3(current.normal, current_camera);
    const double next_camera_dot = dot3(next.normal, next_camera);
    const double current_cosine =
        current_camera_dot / (vision_path_generator::norm(current.normal) * vision_path_generator::norm(current_camera));
    const double next_cosine =
        next_camera_dot / (vision_path_generator::norm(next.normal) * vision_path_generator::norm(next_camera));
    const auto leaving = tangentAt(waypoints, index);
    const auto next_leaving = tangentAt(waypoints, index + 1);
    const double tangent_cosine = dot3(leaving, next_leaving);
    const double segment = vision_path_generator::norm(next.position - current.position);
    discontinuities << problem << ',' << (problem + 1) << ',' << pair_dot << ',' << angleDeg(pair_dot) << ','
                    << segment << ',' << angleDeg(tangent_cosine) << ',' << current_camera_dot << ','
                    << next_camera_dot << ',' << current_cosine << ',' << next_cosine << '\n';

    const double previous_pair = index == 0 ? 0.0 : dot3(waypoints[index - 1].normal, current.normal);
    const double next_pair =
        index + 2 < waypoints.size() ? dot3(next.normal, waypoints[index + 2].normal) : 0.0;
    const auto emit_sign = [&](const char* which, double camera, double previous, double pair, double following) {
      signs << problem << ',' << which << ',' << camera << ',' << previous << ',' << pair << ',' << following
            << '\n';
    };
    emit_sign("none", current_camera_dot, previous_pair, pair_dot, next_pair);
    emit_sign("index", -current_camera_dot, -previous_pair, -pair_dot, next_pair);
    emit_sign("next", -next_camera_dot, previous_pair, -pair_dot, -next_pair);

    std::vector<float> source_distances;
    const auto source_neighbors = nearestPoints(tree, current.position, 1, &source_distances);
    const vision_path_generator::Vec3 source =
        source_neighbors.empty() ? current.position : source_neighbors.front();
    const double nearest_distance = source_distances.empty() ? 0.0 : source_distances.front();
    std::vector<float> k_distances;
    const auto k_neighbors = nearestPoints(tree, source, params.normal_k, &k_distances);
    const auto waypoint_neighbors = nearestPoints(tree, current.position, params.normal_k, nullptr);
    const auto viz = radiusPoints(tree, current.position, kVizRadius);
    writeSurface(output_dir + "/index-" + indexName(problem) + "-surface.csv", k_neighbors, k_distances, viz);

    const auto source_pca = analyzeNeighbors(k_neighbors, source);
    const auto waypoint_pca = analyzeNeighbors(waypoint_neighbors, current.position);
    const double waypoint_angle = angleDeg(dot3(unit(waypoint_pca.normal), unit(current.normal)));
    const double source_angle = angleDeg(dot3(unit(source_pca.normal), unit(current.normal)));
    const double source_variation =
        (source_pca.eigenvalue_0 + source_pca.eigenvalue_1 + source_pca.eigenvalue_2) > 0.0
            ? source_pca.eigenvalue_0 /
                  (source_pca.eigenvalue_0 + source_pca.eigenvalue_1 + source_pca.eigenvalue_2)
            : 0.0;
    const double waypoint_variation =
        (waypoint_pca.eigenvalue_0 + waypoint_pca.eigenvalue_1 + waypoint_pca.eigenvalue_2) > 0.0
            ? waypoint_pca.eigenvalue_0 /
                  (waypoint_pca.eigenvalue_0 + waypoint_pca.eigenvalue_1 + waypoint_pca.eigenvalue_2)
            : 0.0;
    const double planarity =
        waypoint_pca.eigenvalue_2 > 0.0
            ? (waypoint_pca.eigenvalue_1 - waypoint_pca.eigenvalue_0) / waypoint_pca.eigenvalue_2
            : 0.0;
    geometry << problem << ',' << nearest_distance << ',' << waypoint_pca.count << ','
             << waypoint_pca.centroid.x << ',' << waypoint_pca.centroid.y << ',' << waypoint_pca.centroid.z
             << ',' << waypoint_pca.eigenvalue_0 << ',' << waypoint_pca.eigenvalue_1 << ','
             << waypoint_pca.eigenvalue_2 << ',' << waypoint_variation << ',' << planarity << ','
             << waypoint_pca.normal.x << ',' << waypoint_pca.normal.y << ',' << waypoint_pca.normal.z << ','
             << waypoint_pca.camera_dot << ',' << waypoint_angle << ',' << source_pca.count << ','
             << source_pca.eigenvalue_0 << ',' << source_pca.eigenvalue_1 << ',' << source_pca.eigenvalue_2
             << ',' << source_variation << ',' << source_angle << ',' << viz.size() << '\n';
  }

  std::cout << "waypoints " << waypoints.size() << " method " << result.method << "\n";
  return 0;
}
