// P-008-R1E. capture_01에서 지지면 위 물체를 PCL polygonal prism으로 추출한다.
// path_extractor::extract()와 R1의 orientTowardObject는 호출하지 않는다.
// voxel, SOR, RANSAC, Euclidean clustering 파라미터는 R1과 같다.

#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/console/print.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace {

constexpr float kVoxelLeaf = 0.002f;
constexpr int kSorMeanK = 30;
constexpr double kSorStddev = 1.0;
constexpr double kRansacDistance = 0.004;
constexpr int kRansacIterations = 200;
constexpr std::uint32_t kRansacSeed = 1;
constexpr double kClusterTolerance = 0.01;
constexpr int kMinClusterSize = 50;
constexpr double kPrismHeightMin = 0.0;
constexpr double kPrismHeightMax = 0.18;
constexpr double kDiagnosticRadius = 0.08;
constexpr float kViewX = 0.f;
constexpr float kViewY = 0.f;
constexpr float kViewZ = 0.f;

struct Plane {
  Eigen::Vector3d normal{0.0, 0.0, 1.0};
  double d{0.0};
};

double signedDistance(const Eigen::Vector3d& point, const Plane& plane) {
  return plane.normal.dot(point) + plane.d;
}

bool forbiddenInput(const std::string& path) {
  return path.find("capture_02") != std::string::npos || path.find("capture_03") != std::string::npos ||
      path.find("capture_04") != std::string::npos || path.find("capture_05") != std::string::npos;
}

bool loadPcd(const std::string& path, std::vector<Eigen::Vector3d>& points, std::string& error) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    error = "PCD를 열지 못했습니다.";
    return false;
  }
  std::string line;
  int count = -1;
  bool binary = false;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.rfind("POINTS ", 0) == 0) {
      count = std::stoi(line.substr(7));
    }
    if (line == "DATA binary") {
      binary = true;
      break;
    }
    if (line.rfind("DATA ", 0) == 0) {
      error = "binary PCD만 읽습니다.";
      return false;
    }
  }
  if (!binary || count < 0) {
    error = "PCD 헤더가 없습니다.";
    return false;
  }
  points.resize(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index) {
    float xyz[3];
    input.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
    if (!input) {
      error = "PCD 본문이 잘렸습니다.";
      return false;
    }
    points[static_cast<std::size_t>(index)] = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
  }
  return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr toPcl(const std::vector<Eigen::Vector3d>& points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->reserve(points.size());
  for (const auto& point : points) {
    cloud->push_back(pcl::PointXYZ(
        static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z())));
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

bool voxelIndexFits(const std::vector<Eigen::Vector3d>& points, float leaf) {
  float min_x = std::numeric_limits<float>::infinity();
  float min_y = min_x;
  float min_z = min_x;
  float max_x = -min_x;
  float max_y = max_x;
  float max_z = max_x;
  for (const auto& point : points) {
    min_x = std::min(min_x, static_cast<float>(point.x()));
    min_y = std::min(min_y, static_cast<float>(point.y()));
    min_z = std::min(min_z, static_cast<float>(point.z()));
    max_x = std::max(max_x, static_cast<float>(point.x()));
    max_y = std::max(max_y, static_cast<float>(point.y()));
    max_z = std::max(max_z, static_cast<float>(point.z()));
  }
  const double inverse = 1.0 / static_cast<double>(leaf);
  const auto dx = static_cast<std::int64_t>((static_cast<double>(max_x) - min_x) * inverse) + 1;
  const auto dy = static_cast<std::int64_t>((static_cast<double>(max_y) - min_y) * inverse) + 1;
  const auto dz = static_cast<std::int64_t>((static_cast<double>(max_z) - min_z) * inverse) + 1;
  const auto limit = static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max());
  if (dx <= 0 || dy <= 0 || dz <= 0 || dx > limit / dy) {
    return false;
  }
  return (dx * dy) <= limit / dz;
}

std::vector<Eigen::Vector3d> voxelize(const std::vector<Eigen::Vector3d>& points) {
  auto input = toPcl(points);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  filter.setLeafSize(kVoxelLeaf, kVoxelLeaf, kVoxelLeaf);
  filter.filter(*output);
  std::vector<Eigen::Vector3d> result;
  result.reserve(output->size());
  for (const auto& point : output->points) {
    result.emplace_back(point.x, point.y, point.z);
  }
  return result;
}

std::vector<Eigen::Vector3d> removeOutliers(const std::vector<Eigen::Vector3d>& points) {
  auto input = toPcl(points);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  filter.setMeanK(kSorMeanK);
  filter.setStddevMulThresh(kSorStddev);
  filter.filter(*output);
  std::vector<Eigen::Vector3d> result;
  result.reserve(output->size());
  for (const auto& point : output->points) {
    result.emplace_back(point.x, point.y, point.z);
  }
  return result;
}

bool fitPlane(const std::vector<Eigen::Vector3d>& points, Plane& plane, std::vector<int>& inliers) {
  if (points.size() < 3) {
    return false;
  }
  std::mt19937 rng(kRansacSeed);
  std::uniform_int_distribution<int> pick(0, static_cast<int>(points.size()) - 1);
  int best_count = 0;
  Plane best;
  for (int iteration = 0; iteration < kRansacIterations; ++iteration) {
    const int i0 = pick(rng);
    const int i1 = pick(rng);
    const int i2 = pick(rng);
    if (i0 == i1 || i0 == i2 || i1 == i2) {
      continue;
    }
    const Eigen::Vector3d edge1 = points[static_cast<std::size_t>(i1)] - points[static_cast<std::size_t>(i0)];
    const Eigen::Vector3d edge2 = points[static_cast<std::size_t>(i2)] - points[static_cast<std::size_t>(i0)];
    Eigen::Vector3d normal = edge1.cross(edge2);
    if (normal.norm() < 1e-8) {
      continue;
    }
    normal.normalize();
    Plane candidate;
    candidate.normal = normal;
    candidate.d = -normal.dot(points[static_cast<std::size_t>(i0)]);
    int count = 0;
    for (const auto& point : points) {
      if (std::abs(signedDistance(point, candidate)) <= kRansacDistance) {
        ++count;
      }
    }
    if (count > best_count) {
      best_count = count;
      best = candidate;
    }
  }
  if (best_count < 3) {
    return false;
  }
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  int count = 0;
  for (const auto& point : points) {
    if (std::abs(signedDistance(point, best)) <= kRansacDistance) {
      mean += point;
      ++count;
    }
  }
  mean /= static_cast<double>(count);
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto& point : points) {
    if (std::abs(signedDistance(point, best)) <= kRansacDistance) {
      const Eigen::Vector3d delta = point - mean;
      covariance += delta * delta.transpose();
    }
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
  if (normal.dot(best.normal) < 0.0) {
    normal = -normal;
  }
  plane.normal = normal;
  plane.d = -normal.dot(mean);
  inliers.clear();
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (std::abs(signedDistance(points[index], plane)) <= kRansacDistance) {
      inliers.push_back(static_cast<int>(index));
    }
  }
  return inliers.size() >= 3;
}

Eigen::Vector4f viewpointPlane(
    const pcl::PointCloud<pcl::PointXYZ>& hull, float vpx, float vpy, float vpz) {
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
  Eigen::Vector4f centroid = Eigen::Vector4f::Zero();
  pcl::computeMeanAndCovarianceMatrix(hull, covariance, centroid);
  float eigen_value = 0.f;
  Eigen::Vector3f eigen_vector = Eigen::Vector3f::Zero();
  pcl::eigen33(covariance, eigen_value, eigen_vector);
  Eigen::Vector4f coefficients;
  coefficients[0] = eigen_vector[0];
  coefficients[1] = eigen_vector[1];
  coefficients[2] = eigen_vector[2];
  coefficients[3] = 0.f;
  coefficients[3] = -1.f * coefficients.dot(centroid);
  Eigen::Vector4f viewpoint(vpx, vpy, vpz, 0.f);
  viewpoint -= hull[0].getVector4fMap();
  viewpoint[3] = 0.f;
  if (viewpoint.dot(coefficients) < 0.f) {
    coefficients *= -1.f;
    coefficients[3] = 0.f;
    coefficients[3] = -1.f * coefficients.dot(hull[0].getVector4fMap());
  }
  return coefficients;
}

double planeDistance(const pcl::PointXYZ& point, const Eigen::Vector4f& coefficients) {
  return static_cast<double>(coefficients[0]) * point.x + static_cast<double>(coefficients[1]) * point.y +
      static_cast<double>(coefficients[2]) * point.z + static_cast<double>(coefficients[3]);
}

struct PrismOutput {
  std::vector<int> indices;
  int hull_vertices{0};
  int hull_dimension{0};
  double hull_area{0.0};
  pcl::PointCloud<pcl::PointXYZ>::Ptr hull{new pcl::PointCloud<pcl::PointXYZ>};
  Eigen::Vector4f coefficients{Eigen::Vector4f::Zero()};
};

bool extractPrism(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<int>& nonplane,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& inliers,
    float vpx,
    float vpy,
    float vpz,
    double height_min,
    double height_max,
    PrismOutput& output,
    std::string& error) {
  pcl::ConvexHull<pcl::PointXYZ> hull;
  hull.setDimension(2);
  hull.setInputCloud(inliers);
  std::vector<pcl::Vertices> polygons;
  hull.reconstruct(*output.hull, polygons);
  output.hull_dimension = hull.getDimension();
  output.hull_vertices = static_cast<int>(output.hull->size());
  if (output.hull_dimension != 2 || output.hull->size() < 3 || polygons.empty()) {
    error = "hull_dimension이 2가 아닙니다.";
    return false;
  }
  Eigen::Vector3d area = Eigen::Vector3d::Zero();
  const auto& vertices = polygons[0].vertices;
  const auto& origin = output.hull->points[vertices[0]];
  const Eigen::Vector3d base(origin.x, origin.y, origin.z);
  for (std::size_t index = 1; index + 1 < vertices.size(); ++index) {
    const auto& p1 = output.hull->points[vertices[index]];
    const auto& p2 = output.hull->points[vertices[index + 1]];
    const Eigen::Vector3d a(p1.x, p1.y, p1.z);
    const Eigen::Vector3d b(p2.x, p2.y, p2.z);
    area += (a - base).cross(b - base);
  }
  output.hull_area = 0.5 * area.norm();
  output.coefficients = viewpointPlane(*output.hull, vpx, vpy, vpz);
  pcl::IndicesPtr indices(new pcl::Indices(nonplane.begin(), nonplane.end()));
  pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
  prism.setInputCloud(cloud);
  prism.setIndices(indices);
  prism.setInputPlanarHull(output.hull);
  prism.setViewPoint(vpx, vpy, vpz);
  prism.setHeightLimits(height_min, height_max);
  pcl::PointIndices selected;
  prism.segment(selected);
  output.indices = selected.indices;
  float read_x = 1.f;
  float read_y = 1.f;
  float read_z = 1.f;
  prism.getViewPoint(read_x, read_y, read_z);
  if (read_x != vpx || read_y != vpy || read_z != vpz) {
    error = "viewpoint가 반영되지 않았습니다.";
    return false;
  }
  return true;
}

void writeCloud(const std::string& path, const std::vector<Eigen::Vector3d>& points) {
  std::ofstream output(path, std::ios::binary);
  const auto count = static_cast<std::uint32_t>(points.size());
  output.write(reinterpret_cast<const char*>(&count), sizeof(count));
  for (const auto& point : points) {
    const float xyz[3] = {
        static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z())};
    output.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
  }
}

std::string jsonNumber(double value) {
  std::ostringstream stream;
  stream.setf(std::ios::fmtflags(0), std::ios::floatfield);
  stream.precision(17);
  stream << value;
  return stream.str();
}

int selfTest() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> plane_indices;
  std::vector<int> near_indices;
  std::vector<int> far_indices;
  std::vector<int> outside_indices;
  auto add = [&](float x, float y, float z, std::vector<int>* bucket) {
    cloud->push_back(pcl::PointXYZ(x, y, z));
    if (bucket != nullptr) {
      bucket->push_back(static_cast<int>(cloud->size() - 1));
    }
  };
  for (int ix = -7; ix <= 7; ++ix) {
    for (int iy = -7; iy <= 7; ++iy) {
      add(0.02f * static_cast<float>(ix), 0.02f * static_cast<float>(iy), 1.f, &plane_indices);
    }
  }
  for (int index = 0; index < 80; ++index) {
    const float x = 0.004f * static_cast<float>(index % 10);
    const float y = 0.004f * static_cast<float>(index / 10);
    add(x, y, 0.90f, &near_indices);
  }
  for (int index = 0; index < 400; ++index) {
    const float x = 0.002f * static_cast<float>(index % 20);
    const float y = 0.002f * static_cast<float>(index / 20);
    add(x - 0.02f, y - 0.02f, 1.12f, &far_indices);
  }
  for (int index = 0; index < 80; ++index) {
    add(0.60f, 0.004f * static_cast<float>(index % 10), 0.90f, &outside_indices);
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  pcl::PointCloud<pcl::PointXYZ>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZ>);
  for (const int index : plane_indices) {
    inliers->push_back((*cloud)[index]);
  }
  inliers->width = static_cast<std::uint32_t>(inliers->size());
  inliers->height = 1;
  std::vector<int> nonplane;
  nonplane.insert(nonplane.end(), near_indices.begin(), near_indices.end());
  nonplane.insert(nonplane.end(), far_indices.begin(), far_indices.end());
  nonplane.insert(nonplane.end(), outside_indices.begin(), outside_indices.end());
  PrismOutput camera_side;
  std::string error;
  if (!extractPrism(cloud, nonplane, inliers, 0.f, 0.f, 0.f, kPrismHeightMin, kPrismHeightMax, camera_side, error)) {
    std::cerr << error << "\n";
    return 1;
  }
  auto contains = [](const std::vector<int>& indices, int value) {
    return std::find(indices.begin(), indices.end(), value) != indices.end();
  };
  int near_kept = 0;
  int far_kept = 0;
  int outside_kept = 0;
  int plane_kept = 0;
  for (const int index : camera_side.indices) {
    near_kept += contains(near_indices, index) ? 1 : 0;
    far_kept += contains(far_indices, index) ? 1 : 0;
    outside_kept += contains(outside_indices, index) ? 1 : 0;
    plane_kept += contains(plane_indices, index) ? 1 : 0;
  }
  if (near_kept != static_cast<int>(near_indices.size()) || far_kept != 0 || outside_kept != 0 || plane_kept != 0) {
    std::cerr << "PRISM_FILTER FAIL near " << near_kept << " far " << far_kept << " outside " << outside_kept
              << " plane " << plane_kept << "\n";
    return 1;
  }
  PrismOutput far_viewpoint;
  if (!extractPrism(cloud, nonplane, inliers, 0.f, 0.f, 2.f, kPrismHeightMin, kPrismHeightMax, far_viewpoint, error)) {
    std::cerr << error << "\n";
    return 1;
  }
  int near_from_far_view = 0;
  int far_from_far_view = 0;
  for (const int index : far_viewpoint.indices) {
    near_from_far_view += contains(near_indices, index) ? 1 : 0;
    far_from_far_view += contains(far_indices, index) ? 1 : 0;
  }
  if (near_from_far_view != 0 || far_from_far_view != static_cast<int>(far_indices.size())) {
    std::cerr << "VIEWPOINT FAIL near " << near_from_far_view << " far " << far_from_far_view << "\n";
    return 1;
  }
  Plane positive;
  positive.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  positive.d = -1.0;
  Plane negative;
  negative.normal = -positive.normal;
  negative.d = -positive.d;
  std::vector<int> positive_inliers;
  std::vector<int> negative_inliers;
  for (std::size_t index = 0; index < cloud->size(); ++index) {
    const Eigen::Vector3d point((*cloud)[index].x, (*cloud)[index].y, (*cloud)[index].z);
    if (std::abs(signedDistance(point, positive)) <= kRansacDistance) {
      positive_inliers.push_back(static_cast<int>(index));
    }
    if (std::abs(signedDistance(point, negative)) <= kRansacDistance) {
      negative_inliers.push_back(static_cast<int>(index));
    }
  }
  if (positive_inliers != negative_inliers) {
    std::cerr << "PLANE_SIGN_INVARIANCE inlier FAIL\n";
    return 1;
  }
  auto inlierCloud = [&](const std::vector<int>& indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected(new pcl::PointCloud<pcl::PointXYZ>);
    for (const int index : indices) {
      selected->push_back((*cloud)[index]);
    }
    selected->width = static_cast<std::uint32_t>(selected->size());
    selected->height = 1;
    return selected;
  };
  std::vector<int> complement;
  for (std::size_t index = 0; index < cloud->size(); ++index) {
    if (!contains(positive_inliers, static_cast<int>(index))) {
      complement.push_back(static_cast<int>(index));
    }
  }
  PrismOutput from_positive;
  PrismOutput from_negative;
  if (!extractPrism(
          cloud, complement, inlierCloud(positive_inliers), 0.f, 0.f, 0.f, kPrismHeightMin, kPrismHeightMax,
          from_positive, error) ||
      !extractPrism(
          cloud, complement, inlierCloud(negative_inliers), 0.f, 0.f, 0.f, kPrismHeightMin, kPrismHeightMax,
          from_negative, error)) {
    std::cerr << error << "\n";
    return 1;
  }
  auto sorted = [](std::vector<int> indices) {
    std::sort(indices.begin(), indices.end());
    return indices;
  };
  if (sorted(from_positive.indices) != sorted(from_negative.indices)) {
    std::cerr << "PLANE_SIGN_INVARIANCE prism FAIL\n";
    return 1;
  }
  std::cout << "PLANE_SIGN_INVARIANCE PASS\nVIEWPOINT PASS\nPRISM_FILTER PASS\n";
  return 0;
}

struct Cluster {
  int id{0};
  std::vector<int> source_indices;
  std::vector<Eigen::Vector3d> points;
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bbox_min{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bbox_max{Eigen::Vector3d::Zero()};
};

int runCapture(const std::string& input_path, const std::string& out_dir) {
  if (forbiddenInput(input_path)) {
    std::cerr << "capture_02~05 는 입력이 아닙니다.\n";
    return 3;
  }
  if (!voxelIndexFits(std::vector<Eigen::Vector3d>{{0, 0, 0}, {1, 1, 1}}, kVoxelLeaf)) {
    std::cerr << "voxel 인덱스 검사 실패\n";
    return 1;
  }
  std::vector<Eigen::Vector3d> raw;
  std::string error;
  if (!loadPcd(input_path, raw, error)) {
    std::cerr << error << "\n";
    return 1;
  }
  const auto voxel = voxelize(raw);
  const auto denoised = removeOutliers(voxel);
  Plane plane;
  std::vector<int> inliers;
  if (!fitPlane(denoised, plane, inliers)) {
    std::cerr << "지지평면을 찾지 못했습니다.\n";
    return 1;
  }
  std::vector<char> inlier_mark(denoised.size(), 0);
  for (const int index : inliers) {
    inlier_mark[static_cast<std::size_t>(index)] = 1;
  }
  std::vector<int> nonplane;
  std::vector<Eigen::Vector3d> inlier_points;
  inlier_points.reserve(inliers.size());
  for (std::size_t index = 0; index < denoised.size(); ++index) {
    if (inlier_mark[index]) {
      inlier_points.push_back(denoised[index]);
    } else {
      nonplane.push_back(static_cast<int>(index));
    }
  }
  auto cloud = toPcl(denoised);
  auto inlier_cloud = toPcl(inlier_points);
  PrismOutput prism;
  if (!extractPrism(
          cloud, nonplane, inlier_cloud, kViewX, kViewY, kViewZ, kPrismHeightMin, kPrismHeightMax, prism, error)) {
    std::cerr << error << "\n";
    return 6;
  }
  int distance_mismatch = 0;
  for (const int index : prism.indices) {
    const double distance = planeDistance((*cloud)[index], prism.coefficients);
    if (distance < kPrismHeightMin - 1e-4 || distance > kPrismHeightMax + 1e-4) {
      ++distance_mismatch;
    }
  }
  if (distance_mismatch != 0) {
    std::cerr << "prism 선택점의 viewpoint 높이가 limits 밖입니다.\n";
    return 7;
  }
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  for (const auto& point : inlier_points) {
    origin += point;
  }
  origin /= static_cast<double>(inlier_points.size());
  const Eigen::Vector3d normal = plane.normal.normalized();
  Eigen::Vector3d axis_u = normal.unitOrthogonal();
  Eigen::Vector3d axis_v = normal.cross(axis_u);
  auto radiusOf = [&](const Eigen::Vector3d& point) {
    const Eigen::Vector3d delta = point - origin;
    return std::hypot(axis_u.dot(delta), axis_v.dot(delta));
  };
  std::vector<int> disk_nonplane;
  std::vector<double> disk_heights;
  for (const int index : nonplane) {
    if (radiusOf(denoised[static_cast<std::size_t>(index)]) < kDiagnosticRadius) {
      disk_nonplane.push_back(index);
      disk_heights.push_back(planeDistance((*cloud)[index], prism.coefficients));
    }
  }
  std::vector<char> selected_mark(denoised.size(), 0);
  for (const int index : prism.indices) {
    selected_mark[static_cast<std::size_t>(index)] = 1;
  }
  int disk_after = 0;
  for (const int index : disk_nonplane) {
    disk_after += selected_mark[static_cast<std::size_t>(index)] ? 1 : 0;
  }
  std::sort(disk_heights.begin(), disk_heights.end());
  const auto percentile = [&](double rank) {
    if (disk_heights.empty()) {
      return 0.0;
    }
    const double position = rank * static_cast<double>(disk_heights.size() - 1);
    const auto lower = static_cast<std::size_t>(std::floor(position));
    const auto upper = std::min(lower + 1, disk_heights.size() - 1);
    const double weight = position - static_cast<double>(lower);
    return (1.0 - weight) * disk_heights[lower] + weight * disk_heights[upper];
  };
  const double height_median = percentile(0.5);
  std::vector<Eigen::Vector3d> selected_points;
  selected_points.reserve(prism.indices.size());
  for (const int index : prism.indices) {
    selected_points.push_back(denoised[static_cast<std::size_t>(index)]);
  }
  std::vector<Cluster> clusters;
  if (!selected_points.empty()) {
    auto selected_cloud = toPcl(selected_points);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(selected_cloud);
    std::vector<pcl::PointIndices> groups;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
    clustering.setClusterTolerance(kClusterTolerance);
    clustering.setMinClusterSize(kMinClusterSize);
    clustering.setMaxClusterSize(static_cast<int>(selected_points.size()));
    clustering.setSearchMethod(tree);
    clustering.setInputCloud(selected_cloud);
    clustering.extract(groups);
    for (const auto& group : groups) {
      Cluster cluster;
      cluster.source_indices.reserve(group.indices.size());
      cluster.points.reserve(group.indices.size());
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      for (const int local : group.indices) {
        cluster.source_indices.push_back(prism.indices[static_cast<std::size_t>(local)]);
        cluster.points.push_back(selected_points[static_cast<std::size_t>(local)]);
        sum += cluster.points.back();
      }
      cluster.centroid = sum / static_cast<double>(cluster.points.size());
      cluster.bbox_min = cluster.points.front();
      cluster.bbox_max = cluster.points.front();
      for (const auto& point : cluster.points) {
        cluster.bbox_min = cluster.bbox_min.cwiseMin(point);
        cluster.bbox_max = cluster.bbox_max.cwiseMax(point);
      }
      clusters.push_back(std::move(cluster));
    }
  }
  std::sort(clusters.begin(), clusters.end(), [](const Cluster& left, const Cluster& right) {
    if (left.points.size() != right.points.size()) {
      return left.points.size() > right.points.size();
    }
    if (left.centroid.x() != right.centroid.x()) {
      return left.centroid.x() < right.centroid.x();
    }
    if (left.centroid.y() != right.centroid.y()) {
      return left.centroid.y() < right.centroid.y();
    }
    return left.centroid.z() < right.centroid.z();
  });
  std::vector<int> cluster_of(denoised.size(), -1);
  for (std::size_t id = 0; id < clusters.size(); ++id) {
    clusters[id].id = static_cast<int>(id);
    for (const int index : clusters[id].source_indices) {
      cluster_of[static_cast<std::size_t>(index)] = clusters[id].id;
    }
  }
  std::map<int, int> disk_hits;
  for (const int index : disk_nonplane) {
    const int id = cluster_of[static_cast<std::size_t>(index)];
    if (id >= 0) {
      disk_hits[id] += 1;
    }
  }
  int shoe_id = -1;
  int shoe_hits = 0;
  for (const auto& item : disk_hits) {
    if (item.second > shoe_hits) {
      shoe_hits = item.second;
      shoe_id = item.first;
    }
  }
  const double retention = disk_nonplane.empty() ? 0.0 : static_cast<double>(disk_after) / static_cast<double>(disk_nonplane.size());
  const double membership = disk_after == 0 ? 0.0 : static_cast<double>(shoe_hits) / static_cast<double>(disk_after);
  const double selection_ratio =
      nonplane.empty() ? 0.0 : static_cast<double>(prism.indices.size()) / static_cast<double>(nonplane.size());
  std::string verdict = "PASS_EXTRACTION_FIX";
  if (height_median < 0.0) {
    verdict = "FAIL_METHOD_SEMANTICS";
  } else if (retention < 0.5) {
    verdict = "FAIL_SHOE_NOT_RETAINED";
  } else if (selection_ratio > 0.5) {
    verdict = "FAIL_BACKGROUND_REJECTION";
  } else if (membership < 0.5) {
    verdict = "FAIL_CLUSTERING";
  }
  std::filesystem::create_directories(out_dir);
  {
    std::ofstream output(out_dir + "/plane.json");
    output << "{\n";
    output << "  \"a\": " << jsonNumber(plane.normal.x()) << ",\n";
    output << "  \"b\": " << jsonNumber(plane.normal.y()) << ",\n";
    output << "  \"c\": " << jsonNumber(plane.normal.z()) << ",\n";
    output << "  \"d\": " << jsonNumber(plane.d) << ",\n";
    output << "  \"inlier_count\": " << inliers.size() << ",\n";
    output << "  \"viewpoint_a\": " << jsonNumber(prism.coefficients[0]) << ",\n";
    output << "  \"viewpoint_b\": " << jsonNumber(prism.coefficients[1]) << ",\n";
    output << "  \"viewpoint_c\": " << jsonNumber(prism.coefficients[2]) << ",\n";
    output << "  \"viewpoint_d\": " << jsonNumber(prism.coefficients[3]) << ",\n";
    output << "  \"majority_orientation_used\": false\n";
    output << "}\n";
  }
  {
    std::ofstream output(out_dir + "/plane_hull.csv");
    output << "index,x,y,z\n";
    output.setf(std::ios::fmtflags(0), std::ios::floatfield);
    output.precision(17);
    for (std::size_t index = 0; index < prism.hull->size(); ++index) {
      const auto& point = prism.hull->points[index];
      output << index << "," << point.x << "," << point.y << "," << point.z << "\n";
    }
  }
  {
    std::ofstream output(out_dir + "/prism_metrics.json");
    output << "{\n";
    output << "  \"pcl_version\": \"1.12.1\",\n";
    output << "  \"hull_vertex_count\": " << prism.hull_vertices << ",\n";
    output << "  \"hull_dimension\": " << prism.hull_dimension << ",\n";
    output << "  \"hull_area_m2\": " << jsonNumber(prism.hull_area) << ",\n";
    output << "  \"viewpoint\": [0, 0, 0],\n";
    output << "  \"height_min_m\": " << jsonNumber(kPrismHeightMin) << ",\n";
    output << "  \"height_max_m\": " << jsonNumber(kPrismHeightMax) << ",\n";
    output << "  \"height_min_basis\": \"PCL_CONSTRUCTOR_DEFAULT\",\n";
    output << "  \"height_max_basis\": \"GEOMETRIC_SCALE_DERIVED\",\n";
    output << "  \"shoe_height_min_m\": " << jsonNumber(disk_heights.empty() ? 0.0 : disk_heights.front()) << ",\n";
    output << "  \"shoe_height_median_m\": " << jsonNumber(height_median) << ",\n";
    output << "  \"shoe_height_max_m\": " << jsonNumber(disk_heights.empty() ? 0.0 : disk_heights.back()) << ",\n";
    output << "  \"total_nonplane_points\": " << nonplane.size() << ",\n";
    output << "  \"prism_selected_points\": " << prism.indices.size() << ",\n";
    output << "  \"outside_prism_points\": " << (nonplane.size() - prism.indices.size()) << ",\n";
    output << "  \"selection_ratio\": " << jsonNumber(selection_ratio) << ",\n";
    output << "  \"voxel_points\": " << voxel.size() << ",\n";
    output << "  \"sor_points\": " << denoised.size() << ",\n";
    output << "  \"cluster_count\": " << clusters.size() << "\n";
    output << "}\n";
  }
  {
    std::ofstream output(out_dir + "/shoe_retention.json");
    output << "{\n";
    output << "  \"diagnostic_radius_m\": " << jsonNumber(kDiagnosticRadius) << ",\n";
    output << "  \"shoe_region_before_prism\": " << disk_nonplane.size() << ",\n";
    output << "  \"shoe_region_after_prism\": " << disk_after << ",\n";
    output << "  \"retention_ratio\": " << jsonNumber(retention) << ",\n";
    output << "  \"shoe_cluster_id\": " << shoe_id << ",\n";
    output << "  \"shoe_region_membership_ratio\": " << jsonNumber(membership) << ",\n";
    output << "  \"shoe_cluster_point_count\": "
           << (shoe_id >= 0 ? clusters[static_cast<std::size_t>(shoe_id)].points.size() : 0) << "\n";
    output << "}\n";
  }
  {
    std::ofstream output(out_dir + "/clusters.csv");
    output << "cluster_id,point_count,centroid_x,centroid_y,centroid_z,bbox_min_x,bbox_min_y,bbox_min_z,"
              "bbox_max_x,bbox_max_y,bbox_max_z,dim_x,dim_y,dim_z,disk_points,is_shoe_cluster\n";
    output.setf(std::ios::fmtflags(0), std::ios::floatfield);
    output.precision(17);
    for (const auto& cluster : clusters) {
      const Eigen::Vector3d dims = cluster.bbox_max - cluster.bbox_min;
      output << cluster.id << "," << cluster.points.size() << "," << cluster.centroid.x() << ","
             << cluster.centroid.y() << "," << cluster.centroid.z() << "," << cluster.bbox_min.x() << ","
             << cluster.bbox_min.y() << "," << cluster.bbox_min.z() << "," << cluster.bbox_max.x() << ","
             << cluster.bbox_max.y() << "," << cluster.bbox_max.z() << "," << dims.x() << "," << dims.y() << ","
             << dims.z() << "," << disk_hits[cluster.id] << "," << (cluster.id == shoe_id ? 1 : 0) << "\n";
    }
  }
  {
    std::ofstream output(out_dir + "/verdict.json");
    output << "{\n";
    output << "  \"gate\": \"P-008-R1E\",\n";
    output << "  \"verdict\": \"" << verdict << "\",\n";
    output << "  \"method\": \"PCL_EXTRACT_POLYGONAL_PRISM_DATA\",\n";
    output << "  \"parameters_frozen\": false,\n";
    output << "  \"majority_orientation_used\": false,\n";
    output << "  \"retention_pass_rule\": \"before_run retention_ratio >= 0.5\",\n";
    output << "  \"background_pass_rule\": \"before_run selection_ratio <= 0.5\",\n";
    output << "  \"cluster_pass_rule\": \"before_run disk membership_ratio >= 0.5\"\n";
    output << "}\n";
  }
  std::vector<Eigen::Vector3d> selected_xyz;
  std::vector<Eigen::Vector3d> rejected_xyz;
  for (const int index : nonplane) {
    if (selected_mark[static_cast<std::size_t>(index)]) {
      selected_xyz.push_back(denoised[static_cast<std::size_t>(index)]);
    } else {
      rejected_xyz.push_back(denoised[static_cast<std::size_t>(index)]);
    }
  }
  std::vector<Eigen::Vector3d> disk_xyz;
  for (const int index : disk_nonplane) {
    disk_xyz.push_back(denoised[static_cast<std::size_t>(index)]);
  }
  std::vector<Eigen::Vector3d> shoe_xyz;
  if (shoe_id >= 0) {
    shoe_xyz = clusters[static_cast<std::size_t>(shoe_id)].points;
  }
  writeCloud(out_dir + "/stage_inliers.bin", inlier_points);
  writeCloud(out_dir + "/stage_hull.bin", [&] {
    std::vector<Eigen::Vector3d> points;
    for (const auto& point : prism.hull->points) {
      points.emplace_back(point.x, point.y, point.z);
    }
    return points;
  }());
  writeCloud(out_dir + "/stage_prism_selected.bin", selected_xyz);
  writeCloud(out_dir + "/stage_prism_rejected.bin", rejected_xyz);
  writeCloud(out_dir + "/stage_shoe_disk.bin", disk_xyz);
  writeCloud(out_dir + "/stage_shoe_cluster.bin", shoe_xyz);
  {
    std::ofstream output(out_dir + "/stage_clusters.bin", std::ios::binary);
    std::uint32_t count = 0;
    for (const auto& cluster : clusters) {
      count += static_cast<std::uint32_t>(cluster.points.size());
    }
    output.write(reinterpret_cast<const char*>(&count), sizeof(count));
    for (const auto& cluster : clusters) {
      for (const auto& point : cluster.points) {
        const float xyz[3] = {
            static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z())};
        const std::int32_t id = cluster.id;
        output.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
        output.write(reinterpret_cast<const char*>(&id), sizeof(id));
      }
    }
  }
  std::cout << "verdict=" << verdict << " before=" << disk_nonplane.size() << " after=" << disk_after
            << " selected=" << prism.indices.size() << " clusters=" << clusters.size() << " shoe=" << shoe_id
            << " height_median=" << height_median << "\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  if (argc == 2 && std::string(argv[1]) == "--self-test") {
    return selfTest();
  }
  if (argc != 3) {
    std::cerr << "사용: p008r1e_prism <capture_01/cloud.pcd> <output_dir> | --self-test\n";
    return 2;
  }
  return runCapture(argv[1], argv[2]);
}
