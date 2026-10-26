// Copyright 2026 cad-less-robotics contributors

#include "vision_path_generator/path_extractor.hpp"

#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace vision_path_generator {
namespace {

constexpr double kRightAngle = 1.5707963267948966;

struct Plane {
  Eigen::Vector3d normal{0.0, 0.0, 1.0};
  double d{0.0};
};

pcl::PointCloud<pcl::PointXYZ>::Ptr makeCloud() {
  return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

void finalize(pcl::PointCloud<pcl::PointXYZ>& cloud) {
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
}

bool finitePoint(const pcl::PointXYZ& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

Vec3 toVec(const pcl::PointXYZ& point) {
  return Vec3{point.x, point.y, point.z};
}

double signedDistance(const pcl::PointXYZ& point, const Plane& plane) {
  return (plane.normal.x() * point.x) + (plane.normal.y() * point.y) + (plane.normal.z() * point.z) +
         plane.d;
}

void validate(const ExtractParams& params) {
  if (!(params.voxel_leaf_size > 0.f) || !(params.waypoint_spacing > 0.0) ||
      !(params.ransac_distance > 0.0) || !(params.sor_stddev > 0.0) ||
      !(params.boundary_radius > 0.0) || !(params.max_segment_gap > 0.0) ||
      !(params.cluster_tolerance > 0.0)) {
    throw std::invalid_argument("거리 파라미터는 0보다 커야 합니다.");
  }
  if (params.sor_mean_k < 3 || params.ransac_iterations < 1 || params.normal_k < 3 ||
      params.min_cluster_size < 3 || params.min_waypoints < 2 || params.smooth_passes < 0 ||
      params.smooth_passes > 20) {
    throw std::invalid_argument("개수 파라미터가 허용 범위를 벗어났습니다.");
  }
  if (!(params.min_height >= 0.0) || !(params.max_plane_inlier_ratio > 0.0) ||
      params.max_plane_inlier_ratio > 1.0) {
    throw std::invalid_argument("평면 파라미터 범위가 잘못되었습니다.");
  }
  if (params.path_mode != "auto" && params.path_mode != "boundary" && params.path_mode != "crest") {
    throw std::invalid_argument("path_mode는 auto, boundary, crest 중 하나여야 합니다.");
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr removeNonFinite(const pcl::PointCloud<pcl::PointXYZ>& input) {
  auto output = makeCloud();
  output->reserve(input.size());
  for (const auto& point : input.points) {
    if (finitePoint(point)) {
      output->push_back(point);
    }
  }
  finalize(*output);
  return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelize(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input, float leaf) {
  auto output = makeCloud();
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  filter.setLeafSize(leaf, leaf, leaf);
  filter.filter(*output);
  return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input, int mean_k, double stddev) {
  auto output = makeCloud();
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  filter.setMeanK(mean_k);
  filter.setStddevMulThresh(stddev);
  filter.filter(*output);
  return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr extractIndices(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input,
    const pcl::PointIndices::ConstPtr& indices,
    bool negative) {
  auto output = makeCloud();
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(input);
  extract.setIndices(indices);
  extract.setNegative(negative);
  extract.filter(*output);
  return output;
}

bool fitPlane(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    const ExtractParams& params,
    Plane& plane,
    pcl::PointIndices::Ptr inliers,
    std::string& error) {
  pcl::SACSegmentation<pcl::PointXYZ> segmentation;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(params.ransac_distance);
  segmentation.setMaxIterations(params.ransac_iterations);
  segmentation.setInputCloud(cloud);
  segmentation.segment(*inliers, *coefficients);
  if (!inliers || inliers->indices.empty() || !coefficients || coefficients->values.size() < 4) {
    error = "RANSAC이 작업대 평면을 찾지 못했습니다.";
    return false;
  }

  Eigen::Vector3d normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  const double length = normal.norm();
  if (length < 1e-9) {
    error = "RANSAC 평면 법선이 0입니다.";
    return false;
  }
  plane.normal = normal / length;
  plane.d = coefficients->values[3] / length;
  return true;
}

void orientTowardObject(Plane& plane, const pcl::PointCloud<pcl::PointXYZ>& object) {
  double sum = 0.0;
  for (const auto& point : object.points) {
    sum += signedDistance(point, plane);
  }
  if (sum < 0.0) {
    plane.normal = -plane.normal;
    plane.d = -plane.d;
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr keepAbove(
    const pcl::PointCloud<pcl::PointXYZ>& cloud, const Plane& plane, double min_height) {
  auto output = makeCloud();
  output->reserve(cloud.size());
  for (const auto& point : cloud.points) {
    if (signedDistance(point, plane) >= min_height) {
      output->push_back(point);
    }
  }
  finalize(*output);
  return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr largestCluster(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const ExtractParams& params, std::string& error) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
  clustering.setClusterTolerance(params.cluster_tolerance);
  clustering.setMinClusterSize(params.min_cluster_size);
  clustering.setMaxClusterSize(static_cast<int>(std::min<std::size_t>(
      cloud->size(), static_cast<std::size_t>(std::numeric_limits<int>::max()))));
  clustering.setSearchMethod(tree);
  clustering.setInputCloud(cloud);
  std::vector<pcl::PointIndices> clusters;
  clustering.extract(clusters);
  if (clusters.empty()) {
    error = "제품 클러스터가 없습니다. min_cluster_size 또는 cluster_tolerance를 확인하세요.";
    return makeCloud();
  }
  const auto best = std::max_element(
      clusters.begin(), clusters.end(), [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
        return a.indices.size() < b.indices.size();
      });
  pcl::PointIndices::Ptr indices(new pcl::PointIndices(*best));
  return extractIndices(cloud, indices, false);
}

pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const ExtractParams& params, std::string& error) {
  if (cloud->size() < 8) {
    error = "법선을 계산할 제품 점이 부족합니다.";
    return pcl::PointCloud<pcl::Normal>::Ptr();
  }
  int k = params.normal_k;
  if (k >= static_cast<int>(cloud->size())) {
    k = static_cast<int>(cloud->size()) - 1;
  }
  if (k < 3) {
    error = "법선 K-NN 이웃이 3개 미만입니다.";
    return pcl::PointCloud<pcl::Normal>::Ptr();
  }
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> estimator;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  estimator.setInputCloud(cloud);
  estimator.setSearchMethod(tree);
  estimator.setKSearch(k);
  auto normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  estimator.compute(*normals);
  if (normals->size() != cloud->size()) {
    error = "법선 개수가 점 개수와 다릅니다.";
    return pcl::PointCloud<pcl::Normal>::Ptr();
  }
  return normals;
}

void alignNormals(pcl::PointCloud<pcl::Normal>& normals, const Plane& plane) {
  for (auto& normal : normals.points) {
    Eigen::Vector3d vector(normal.normal_x, normal.normal_y, normal.normal_z);
    if (!std::isfinite(vector.x()) || vector.norm() < 1e-9) {
      normal.normal_x = 0.f;
      normal.normal_y = 0.f;
      normal.normal_z = 0.f;
      continue;
    }
    if (vector.dot(plane.normal) < 0.0) {
      vector = -vector;
    }
    vector.normalize();
    normal.normal_x = static_cast<float>(vector.x());
    normal.normal_y = static_cast<float>(vector.y());
    normal.normal_z = static_cast<float>(vector.z());
  }
}

std::vector<Vec3> longestGapRun(const std::vector<Vec3>& ordered, double max_gap) {
  std::vector<Vec3> best;
  std::vector<Vec3> current;
  auto commit = [&]() {
    if (current.size() > best.size()) {
      best = current;
    }
  };
  for (const auto& point : ordered) {
    if (!current.empty() && norm(point - current.back()) > max_gap) {
      commit();
      current.clear();
    }
    current.push_back(point);
  }
  commit();
  return best;
}

std::vector<Vec3> boundaryChain(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
    const ExtractParams& params) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary;
  boundary.setInputCloud(cloud);
  boundary.setInputNormals(normals);
  boundary.setSearchMethod(tree);
  boundary.setRadiusSearch(params.boundary_radius);
  boundary.setAngleThreshold(static_cast<float>(kRightAngle));
  pcl::PointCloud<pcl::Boundary> marks;
  boundary.compute(marks);
  std::vector<Vec3> points;
  if (marks.size() != cloud->size()) {
    return points;
  }
  points.reserve(marks.size());
  for (std::size_t index = 0; index < marks.size(); ++index) {
    if (marks[index].boundary_point != 0 && finitePoint(cloud->at(index))) {
      points.push_back(toVec(cloud->at(index)));
    }
  }
  return longestNearestNeighborChain(points, params.max_segment_gap);
}

std::vector<Vec3> crestChain(
    const pcl::PointCloud<pcl::PointXYZ>& cloud, const Plane& plane, const ExtractParams& params) {
  if (cloud.size() < 2) {
    return {};
  }
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto& point : cloud.points) {
    centroid += Eigen::Vector3d(point.x, point.y, point.z);
  }
  centroid /= static_cast<double>(cloud.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto& point : cloud.points) {
    const Eigen::Vector3d delta = Eigen::Vector3d(point.x, point.y, point.z) - centroid;
    covariance += delta * delta.transpose();
  }
  covariance /= static_cast<double>(cloud.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return {};
  }
  const Eigen::Vector3d axis = solver.eigenvectors().col(2);

  double t_min = std::numeric_limits<double>::infinity();
  double t_max = -std::numeric_limits<double>::infinity();
  std::vector<double> projection(cloud.size(), 0.0);
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    const auto& sample = cloud.points[index];
    const Eigen::Vector3d point(sample.x, sample.y, sample.z);
    projection[index] = (point - centroid).dot(axis);
    t_min = std::min(t_min, projection[index]);
    t_max = std::max(t_max, projection[index]);
  }
  const double span = t_max - t_min;
  if (!(span > params.waypoint_spacing)) {
    return {};
  }
  const double bin_count_f = std::floor(span / params.waypoint_spacing) + 1.0;
  if (bin_count_f > 100000.0) {
    return {};
  }
  const auto bin_count = static_cast<std::size_t>(bin_count_f);
  struct Bin {
    bool filled{false};
    double height{-std::numeric_limits<double>::infinity()};
    Vec3 point;
  };
  std::vector<Bin> bins(bin_count);
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    auto bin = static_cast<std::size_t>(
        std::floor((projection[index] - t_min) / params.waypoint_spacing));
    if (bin >= bin_count) {
      bin = bin_count - 1;
    }
    const auto& sample = cloud.points[index];
    const double height = signedDistance(sample, plane);
    if (!bins[bin].filled || height > bins[bin].height) {
      bins[bin].filled = true;
      bins[bin].height = height;
      bins[bin].point = toVec(sample);
    }
  }
  std::vector<Vec3> ordered;
  for (const auto& bin : bins) {
    if (bin.filled) {
      ordered.push_back(bin.point);
    }
  }
  return longestGapRun(ordered, params.max_segment_gap);
}

std::vector<Waypoint> buildWaypoints(
    const std::vector<Vec3>& samples,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
    const Plane& plane,
    std::string& error) {
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(cloud);
  std::vector<Quat> orientations;
  orientations.reserve(samples.size());
  std::size_t missing_normals = 0;
  for (std::size_t index = 0; index < samples.size(); ++index) {
    Vec3 tangent = (index + 1 < samples.size()) ? samples[index + 1] - samples[index]
                                                 : samples[index] - samples[index - 1];
    pcl::PointXYZ query;
    query.x = static_cast<float>(samples[index].x);
    query.y = static_cast<float>(samples[index].y);
    query.z = static_cast<float>(samples[index].z);
    std::vector<int> ids(1);
    std::vector<float> distance(1);
    Eigen::Vector3d normal = plane.normal;
    if (tree.nearestKSearch(query, 1, ids, distance) > 0) {
      const auto& source = normals->at(static_cast<std::size_t>(ids[0]));
      Eigen::Vector3d candidate(source.normal_x, source.normal_y, source.normal_z);
      if (candidate.norm() > 1e-6 && std::isfinite(candidate.x())) {
        normal = candidate.normalized();
      } else {
        ++missing_normals;
      }
    } else {
      ++missing_normals;
    }
    orientations.push_back(orientationFromTangentAndNormal(tangent, Vec3{normal.x(), normal.y(), normal.z()}));
  }
  if (missing_normals * 2 > samples.size()) {
    error = "법선 추정에 실패했습니다.";
    return {};
  }
  alignQuaternionHemisphere(orientations);
  std::vector<Waypoint> waypoints;
  waypoints.reserve(samples.size());
  for (std::size_t index = 0; index < samples.size(); ++index) {
    waypoints.push_back(Waypoint{samples[index], orientations[index]});
  }
  return waypoints;
}

std::vector<Vec3> finishPolyline(std::vector<Vec3> polyline, const ExtractParams& params, std::string& error) {
  if (polyline.size() < params.min_waypoints) {
    error = "컨투어 점이 최소 웨이포인트 수보다 적습니다.";
    return {};
  }
  polyline = smoothOpenPolyline(polyline, params.smooth_passes);
  std::vector<Vec3> sampled = resamplePolyline(polyline, params.waypoint_spacing);
  if (sampled.size() < params.min_waypoints) {
    error = "리샘플된 웨이포인트가 최소 개수보다 적습니다.";
    return {};
  }
  return sampled;
}

}  // namespace

PathExtractor::PathExtractor(ExtractParams params) : params_(std::move(params)) {
  validate(params_);
}

ExtractResult PathExtractor::extract(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) const {
  ExtractResult result;
  result.filtered = makeCloud();
  result.object = makeCloud();
  if (!cloud) {
    result.error = "입력 포인트 클라우드가 없습니다.";
    return result;
  }
  result.input_points = cloud->size();

  try {
    auto finite = removeNonFinite(*cloud);
    if (finite->size() <= static_cast<std::size_t>(params_.sor_mean_k)) {
      result.error = "유효 점이 SOR mean_k 이하입니다.";
      return result;
    }
    auto voxel = voxelize(finite, params_.voxel_leaf_size);
    result.voxel_points = voxel->size();
    if (voxel->size() <= static_cast<std::size_t>(params_.sor_mean_k)) {
      result.error = "다운샘플 후 점이 SOR mean_k 이하입니다.";
      result.filtered = voxel;
      return result;
    }
    auto denoised = removeOutliers(voxel, params_.sor_mean_k, params_.sor_stddev);
    result.sor_points = denoised->size();
    result.filtered = denoised;
    if (denoised->size() < 10) {
      result.error = "노이즈 제거 후 점이 부족합니다.";
      return result;
    }

    Plane plane;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    if (!fitPlane(denoised, params_, plane, inliers, result.error)) {
      return result;
    }
    result.plane_inliers = inliers->indices.size();
    result.plane_inlier_ratio =
        static_cast<double>(result.plane_inliers) / static_cast<double>(denoised->size());
    if (result.plane_inlier_ratio > params_.max_plane_inlier_ratio) {
      result.error = "장면의 대부분이 평면입니다. 제품이 없거나 RANSAC 거리가 너무 큽니다.";
      return result;
    }

    auto without_plane = extractIndices(denoised, inliers, true);
    if (without_plane->empty()) {
      result.error = "평면 제거 후 점이 남지 않았습니다.";
      return result;
    }
    orientTowardObject(plane, *without_plane);
    auto above = keepAbove(*without_plane, plane, params_.min_height);
    if (above->size() < static_cast<std::size_t>(params_.min_cluster_size)) {
      result.object = above;
      result.object_points = above->size();
      result.error = "높이 필터 후 제품 점이 min_cluster_size보다 적습니다.";
      return result;
    }
    auto object = largestCluster(above, params_, result.error);
    result.object = object;
    result.object_points = object->size();
    if (!result.error.empty()) {
      return result;
    }

    auto normals = estimateNormals(object, params_, result.error);
    if (!normals) {
      return result;
    }
    alignNormals(*normals, plane);

    std::vector<Vec3> polyline;
    if (params_.path_mode != "crest") {
      polyline = boundaryChain(object, normals, params_);
      if (polyline.size() >= params_.min_waypoints) {
        result.method = "boundary";
      }
    }
    if (result.method.empty() && params_.path_mode != "boundary") {
      polyline = crestChain(*object, plane, params_);
      result.method = params_.path_mode == "crest" ? "crest" : "crest_fallback";
    }
    if (result.method.empty()) {
      result.error = "경계 컨투어 포인트가 부족합니다.";
      return result;
    }

    auto sampled = finishPolyline(std::move(polyline), params_, result.error);
    if (!result.error.empty()) {
      return result;
    }
    result.waypoints = buildWaypoints(sampled, object, normals, plane, result.error);
    if (!result.error.empty()) {
      result.waypoints.clear();
    }
  } catch (const std::exception& ex) {
    result.error = ex.what();
  }
  return result;
}

}  // namespace vision_path_generator
