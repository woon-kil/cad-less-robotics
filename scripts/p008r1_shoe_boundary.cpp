// P-008-R1. capture_01 점군만 받아 신발 경계를 만든다.
// path_extractor::extract()는 법선과 waypoint까지 가므로 호출하지 않는다.
// voxel, SOR, RANSAC, Euclidean clustering은 그 구현과 같은 파라미터의 PCL 단계다.

#include <pcl/console/print.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
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
constexpr double kMinHeight = 0.003;
constexpr double kClusterTolerance = 0.01;
constexpr int kMinClusterSize = 50;
constexpr double kShoeLengthMin = 0.12;
constexpr double kShoeLengthMax = 0.36;
constexpr double kShoeWidthMin = 0.05;
constexpr double kShoeWidthMax = 0.16;
constexpr double kShoeHeightMin = 0.03;
constexpr double kShoeHeightMax = 0.18;
constexpr double kCentroidHeightMin = 0.01;
constexpr double kCentroidHeightMax = 0.22;
constexpr uint32_t kRansacSeed = 1;

struct Plane {
  Eigen::Vector3d normal{0.0, 0.0, 1.0};
  double d{0.0};
};

struct Cloud {
  std::vector<Eigen::Vector3d> points;
};

struct Cluster {
  int id{0};
  std::vector<Eigen::Vector3d> points;
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bbox_min{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bbox_max{Eigen::Vector3d::Zero()};
  double height_peak{0.0};
  double centroid_height{0.0};
  double length_m{0.0};
  double width_m{0.0};
  bool rule_pass{false};
  bool selected{false};
};

double signedDistance(const Eigen::Vector3d& point, const Plane& plane) {
  return plane.normal.dot(point) + plane.d;
}

bool loadPcd(const std::string& path, Cloud& cloud, std::string& error) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    error = "PCD를 열지 못했습니다.";
    return false;
  }
  std::string line;
  int points = -1;
  bool binary = false;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.rfind("POINTS ", 0) == 0) {
      points = std::stoi(line.substr(7));
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
  if (!binary || points < 0) {
    error = "PCD 헤더에 POINTS 또는 DATA binary가 없습니다.";
    return false;
  }
  cloud.points.resize(static_cast<std::size_t>(points));
  for (int index = 0; index < points; ++index) {
    float xyz[3];
    input.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
    if (!input) {
      error = "PCD 본문이 잘렸습니다.";
      return false;
    }
    cloud.points[static_cast<std::size_t>(index)] = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
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

std::vector<Eigen::Vector3d> fromPcl(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  std::vector<Eigen::Vector3d> points;
  points.reserve(cloud.size());
  for (const auto& point : cloud.points) {
    points.emplace_back(point.x, point.y, point.z);
  }
  return points;
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

std::vector<Eigen::Vector3d> voxelByLeafKey(const std::vector<Eigen::Vector3d>& points, float leaf) {
  float min_x = std::numeric_limits<float>::infinity();
  float min_y = min_x;
  float min_z = min_x;
  for (const auto& point : points) {
    min_x = std::min(min_x, static_cast<float>(point.x()));
    min_y = std::min(min_y, static_cast<float>(point.y()));
    min_z = std::min(min_z, static_cast<float>(point.z()));
  }
  struct Sum {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    int count{0};
  };
  std::map<std::tuple<int, int, int>, Sum> bins;
  const double inverse = 1.0 / static_cast<double>(leaf);
  for (const auto& point : points) {
    const auto key = std::make_tuple(
        static_cast<int>(std::floor((point.x() - min_x) * inverse)),
        static_cast<int>(std::floor((point.y() - min_y) * inverse)),
        static_cast<int>(std::floor((point.z() - min_z) * inverse)));
    auto& sum = bins[key];
    sum.x += point.x();
    sum.y += point.y();
    sum.z += point.z();
    ++sum.count;
  }
  std::vector<Eigen::Vector3d> output;
  output.reserve(bins.size());
  for (const auto& item : bins) {
    const auto& sum = item.second;
    const double scale = 1.0 / static_cast<double>(sum.count);
    output.emplace_back(sum.x * scale, sum.y * scale, sum.z * scale);
  }
  return output;
}

std::vector<Eigen::Vector3d> voxelize(const std::vector<Eigen::Vector3d>& points, bool& fallback) {
  fallback = !voxelIndexFits(points, kVoxelLeaf);
  if (fallback) {
    return voxelByLeafKey(points, kVoxelLeaf);
  }
  auto input = toPcl(points);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  filter.setLeafSize(kVoxelLeaf, kVoxelLeaf, kVoxelLeaf);
  filter.filter(*output);
  return fromPcl(*output);
}

std::vector<Eigen::Vector3d> removeOutliers(const std::vector<Eigen::Vector3d>& points) {
  auto input = toPcl(points);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  filter.setMeanK(kSorMeanK);
  filter.setStddevMulThresh(kSorStddev);
  filter.filter(*output);
  return fromPcl(*output);
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
    int i0 = pick(rng);
    int i1 = pick(rng);
    int i2 = pick(rng);
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

void orientTowardObject(Plane& plane, const std::vector<Eigen::Vector3d>& object) {
  double sum = 0.0;
  for (const auto& point : object) {
    sum += signedDistance(point, plane);
  }
  if (sum < 0.0) {
    plane.normal = -plane.normal;
    plane.d = -plane.d;
  }
}

void bbox(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& min_point, Eigen::Vector3d& max_point) {
  min_point = points.front();
  max_point = points.front();
  for (const auto& point : points) {
    min_point = min_point.cwiseMin(point);
    max_point = max_point.cwiseMax(point);
  }
}

void planarSize(
    const std::vector<Eigen::Vector3d>& points,
    const Plane& plane,
    double& length,
    double& width,
    double& peak,
    double& centroid_height) {
  const Eigen::Vector3d normal = plane.normal.normalized();
  const Eigen::Vector3d axis_u = normal.unitOrthogonal();
  const Eigen::Vector3d axis_v = normal.cross(axis_u);
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  std::vector<Eigen::Vector2d> uv;
  uv.reserve(points.size());
  double height_sum = 0.0;
  peak = -std::numeric_limits<double>::infinity();
  for (const auto& point : points) {
    const double height = signedDistance(point, plane);
    peak = std::max(peak, height);
    height_sum += height;
    const Eigen::Vector2d coord(axis_u.dot(point), axis_v.dot(point));
    uv.push_back(coord);
    mean += coord;
  }
  centroid_height = height_sum / static_cast<double>(points.size());
  mean /= static_cast<double>(points.size());
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const auto& coord : uv) {
    const Eigen::Vector2d delta = coord - mean;
    covariance += delta * delta.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  const Eigen::Vector2d major = solver.eigenvectors().col(1).normalized();
  const Eigen::Vector2d minor = solver.eigenvectors().col(0).normalized();
  double major_min = std::numeric_limits<double>::infinity();
  double major_max = -major_min;
  double minor_min = major_min;
  double minor_max = major_max;
  for (const auto& coord : uv) {
    const double major_value = major.dot(coord);
    const double minor_value = minor.dot(coord);
    major_min = std::min(major_min, major_value);
    major_max = std::max(major_max, major_value);
    minor_min = std::min(minor_min, minor_value);
    minor_max = std::max(minor_max, minor_value);
  }
  length = major_max - major_min;
  width = minor_max - minor_min;
  if (width > length) {
    std::swap(length, width);
  }
}

bool shoeRule(const Cluster& cluster) {
  return cluster.length_m >= kShoeLengthMin && cluster.length_m <= kShoeLengthMax &&
      cluster.width_m >= kShoeWidthMin && cluster.width_m <= kShoeWidthMax &&
      cluster.height_peak >= kShoeHeightMin && cluster.height_peak <= kShoeHeightMax &&
      cluster.centroid_height >= kCentroidHeightMin && cluster.centroid_height <= kCentroidHeightMax;
}

std::vector<Cluster> clusterObjects(const std::vector<Eigen::Vector3d>& points, const Plane& plane) {
  auto input = toPcl(points);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(input);
  std::vector<pcl::PointIndices> groups;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
  clustering.setClusterTolerance(kClusterTolerance);
  clustering.setMinClusterSize(kMinClusterSize);
  clustering.setMaxClusterSize(static_cast<int>(points.size()));
  clustering.setSearchMethod(tree);
  clustering.setInputCloud(input);
  clustering.extract(groups);
  std::vector<Cluster> clusters;
  int next_id = 0;
  for (const auto& group : groups) {
    Cluster cluster;
    cluster.id = next_id++;
    cluster.points.reserve(group.indices.size());
    for (const int index : group.indices) {
      cluster.points.push_back(points[static_cast<std::size_t>(index)]);
    }
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto& point : cluster.points) {
      sum += point;
    }
    cluster.centroid = sum / static_cast<double>(cluster.points.size());
    bbox(cluster.points, cluster.bbox_min, cluster.bbox_max);
    planarSize(cluster.points, plane, cluster.length_m, cluster.width_m, cluster.height_peak, cluster.centroid_height);
    cluster.rule_pass = shoeRule(cluster);
    clusters.push_back(std::move(cluster));
  }
  std::sort(clusters.begin(), clusters.end(), [](const Cluster& a, const Cluster& b) {
    if (a.points.size() != b.points.size()) {
      return a.points.size() > b.points.size();
    }
    if (a.centroid.x() != b.centroid.x()) {
      return a.centroid.x() < b.centroid.x();
    }
    if (a.centroid.y() != b.centroid.y()) {
      return a.centroid.y() < b.centroid.y();
    }
    return a.centroid.z() < b.centroid.z();
  });
  for (std::size_t index = 0; index < clusters.size(); ++index) {
    clusters[index].id = static_cast<int>(index);
  }
  int selected = -1;
  for (std::size_t index = 0; index < clusters.size(); ++index) {
    if (!clusters[index].rule_pass) {
      continue;
    }
    if (selected < 0 || clusters[index].points.size() > clusters[static_cast<std::size_t>(selected)].points.size()) {
      selected = static_cast<int>(index);
    }
  }
  if (selected >= 0) {
    clusters[static_cast<std::size_t>(selected)].selected = true;
  }
  return clusters;
}

struct Frame {
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
  Eigen::Vector3d eigenvalues{Eigen::Vector3d::Zero()};
  Eigen::Vector3d axis0{1.0, 0.0, 0.0};
  Eigen::Vector3d axis1{0.0, 1.0, 0.0};
  Eigen::Vector3d axis2{0.0, 0.0, 1.0};
};

Eigen::Vector3d canonicalAxis(Eigen::Vector3d axis) {
  for (int index = 0; index < 3; ++index) {
    if (std::abs(axis[index]) > 1e-8) {
      if (axis[index] < 0.0) {
        axis = -axis;
      }
      break;
    }
  }
  return axis;
}

Frame pcaFrame(const std::vector<Eigen::Vector3d>& points) {
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto& point : points) {
    centroid += point;
  }
  centroid /= static_cast<double>(points.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto& point : points) {
    const Eigen::Vector3d delta = point - centroid;
    covariance += delta * delta.transpose();
  }
  covariance /= static_cast<double>(points.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  Frame frame;
  frame.centroid = centroid;
  frame.eigenvalues = solver.eigenvalues();
  frame.axis0 = canonicalAxis(solver.eigenvectors().col(2));
  frame.axis2 = solver.eigenvectors().col(0);
  if (frame.axis2.dot(centroid) > 0.0) {
    frame.axis2 = -frame.axis2;
  }
  frame.axis1 = frame.axis2.cross(frame.axis0);
  Eigen::Matrix3d axes;
  axes.col(0) = frame.axis0;
  axes.col(1) = frame.axis1;
  axes.col(2) = frame.axis2;
  if (axes.determinant() < 0.0) {
    frame.axis1 = -frame.axis1;
  }
  return frame;
}

int orient(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c) {
  const double value = (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
  if (std::abs(value) <= 1e-12) {
    return 0;
  }
  return value > 0.0 ? 1 : -1;
}

bool properCross(
    const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c, const Eigen::Vector2d& d) {
  const int o1 = orient(a, b, c);
  const int o2 = orient(a, b, d);
  const int o3 = orient(c, d, a);
  const int o4 = orient(c, d, b);
  return o1 * o2 < 0 && o3 * o4 < 0;
}

int selfIntersections(const std::vector<Eigen::Vector2d>& polygon) {
  const int count = static_cast<int>(polygon.size());
  if (count < 4) {
    return 0;
  }
  int hits = 0;
  for (int i = 0; i < count; ++i) {
    const Eigen::Vector2d a = polygon[static_cast<std::size_t>(i)];
    const Eigen::Vector2d b = polygon[static_cast<std::size_t>((i + 1) % count)];
    for (int j = i + 1; j < count; ++j) {
      if (std::abs(i - j) <= 1 || (i == 0 && j == count - 1)) {
        continue;
      }
      const Eigen::Vector2d c = polygon[static_cast<std::size_t>(j)];
      const Eigen::Vector2d d = polygon[static_cast<std::size_t>((j + 1) % count)];
      if (properCross(a, b, c, d)) {
        ++hits;
      }
    }
  }
  return hits;
}

double shoelace(const std::vector<Eigen::Vector2d>& polygon) {
  double area = 0.0;
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const auto& current = polygon[index];
    const auto& next = polygon[(index + 1) % polygon.size()];
    area += current.x() * next.y() - next.x() * current.y();
  }
  return 0.5 * area;
}

double perimeterOf(const std::vector<Eigen::Vector2d>& polygon) {
  double length = 0.0;
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    length += (polygon[(index + 1) % polygon.size()] - polygon[index]).norm();
  }
  return length;
}

std::vector<Eigen::Vector2d> dedup(const std::vector<Eigen::Vector2d>& polygon, int& removed) {
  std::vector<Eigen::Vector2d> output;
  for (const auto& point : polygon) {
    if (!output.empty() && (output.back() - point).norm() <= 1e-12) {
      ++removed;
      continue;
    }
    output.push_back(point);
  }
  if (output.size() >= 2 && (output.front() - output.back()).norm() <= 1e-12) {
    output.pop_back();
    ++removed;
  }
  return output;
}

struct Hull {
  bool valid{false};
  double alpha{0.0};
  std::vector<Eigen::Vector2d> polygon;
  int loops{0};
  int intersections{0};
  double area{0.0};
  double perimeter{0.0};
  double area_ratio{0.0};
  double perimeter_ratio{0.0};
  std::string reason;
};

Hull makeHull(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double alpha,
    double convex_area,
    double convex_perimeter) {
  Hull hull;
  hull.alpha = alpha;
  try {
    pcl::ConcaveHull<pcl::PointXYZ> concave;
    concave.setDimension(2);
    concave.setAlpha(alpha);
    concave.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZ> output;
    std::vector<pcl::Vertices> polygons;
    concave.reconstruct(output, polygons);
    hull.loops = static_cast<int>(polygons.size());
    if (polygons.size() != 1 || polygons[0].vertices.size() < 3) {
      hull.reason = "loop_count";
      return hull;
    }
    std::vector<Eigen::Vector2d> polygon;
    polygon.reserve(polygons[0].vertices.size());
    for (const auto index : polygons[0].vertices) {
      const auto& point = output[index];
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        hull.reason = "non_finite";
        return hull;
      }
      polygon.emplace_back(point.x, point.y);
    }
    int removed = 0;
    polygon = dedup(polygon, removed);
    if (polygon.size() < 3) {
      hull.reason = "too_short";
      return hull;
    }
    hull.polygon = polygon;
    hull.intersections = selfIntersections(polygon);
    hull.area = shoelace(polygon);
    hull.perimeter = perimeterOf(polygon);
    hull.area_ratio = convex_area > 0.0 ? std::abs(hull.area) / convex_area : 0.0;
    hull.perimeter_ratio = convex_perimeter > 0.0 ? hull.perimeter / convex_perimeter : 0.0;
    if (hull.intersections != 0) {
      hull.reason = "self_intersection";
      return hull;
    }
    hull.valid = true;
    hull.reason = "topology_ok";
    (void)removed;
  } catch (const std::exception& ex) {
    hull.reason = ex.what();
  }
  return hull;
}

void writeCloud(const std::string& path, const std::vector<Eigen::Vector3d>& points) {
  std::ofstream output(path, std::ios::binary);
  const std::uint32_t count = static_cast<std::uint32_t>(points.size());
  output.write(reinterpret_cast<const char*>(&count), sizeof(count));
  for (const auto& point : points) {
    const float xyz[3] = {
        static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z())};
    output.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
  }
}

void writeClusters(const std::string& path, const std::vector<Cluster>& clusters) {
  std::ofstream output(path, std::ios::binary);
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

std::string jsonNumber(double value) {
  std::ostringstream stream;
  stream.setf(std::ios::fmtflags(0), std::ios::floatfield);
  stream.precision(17);
  stream << value;
  return stream.str();
}

bool forbiddenInput(const std::string& path) {
  return path.find("capture_02") != std::string::npos || path.find("capture_03") != std::string::npos ||
      path.find("capture_04") != std::string::npos || path.find("capture_05") != std::string::npos;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "사용: p008r1_shoe_boundary <capture_01/cloud.pcd> <output_dir>\n";
    return 2;
  }
  const std::string input_path = argv[1];
  const std::string out_dir = argv[2];
  if (forbiddenInput(input_path)) {
    std::cerr << "capture_02~05 는 R1 입력이 아닙니다.\n";
    return 3;
  }
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  Cloud raw;
  std::string error;
  if (!loadPcd(input_path, raw, error)) {
    std::cerr << error << "\n";
    return 1;
  }
  bool voxel_fallback = false;
  auto voxel = voxelize(raw.points, voxel_fallback);
  auto denoised = removeOutliers(voxel);
  Plane plane;
  std::vector<int> inlier_index;
  if (!fitPlane(denoised, plane, inlier_index)) {
    std::cerr << "지지평면을 찾지 못했습니다.\n";
    return 1;
  }
  std::vector<char> inlier_mark(denoised.size(), 0);
  for (const int index : inlier_index) {
    inlier_mark[static_cast<std::size_t>(index)] = 1;
  }
  std::vector<Eigen::Vector3d> inliers;
  std::vector<Eigen::Vector3d> without_plane;
  inliers.reserve(inlier_index.size());
  for (std::size_t index = 0; index < denoised.size(); ++index) {
    if (inlier_mark[index]) {
      inliers.push_back(denoised[index]);
    } else {
      without_plane.push_back(denoised[index]);
    }
  }
  orientTowardObject(plane, without_plane);
  std::vector<Eigen::Vector3d> above;
  for (const auto& point : without_plane) {
    if (signedDistance(point, plane) >= kMinHeight) {
      above.push_back(point);
    }
  }
  auto clusters = clusterObjects(above, plane);
  const Cluster* selected = nullptr;
  for (const auto& cluster : clusters) {
    if (cluster.selected) {
      selected = &cluster;
      break;
    }
  }
  writeCloud(out_dir + "/stage_voxel_sor.bin", denoised);
  writeCloud(out_dir + "/stage_plane_inliers.bin", inliers);
  writeCloud(out_dir + "/stage_plane_removed.bin", above);
  writeClusters(out_dir + "/stage_clusters.bin", clusters);

  std::ofstream cluster_csv(out_dir + "/clusters.csv");
  cluster_csv << "cluster_id,point_count,centroid_x,centroid_y,centroid_z,bbox_min_x,bbox_min_y,bbox_min_z,"
                 "bbox_max_x,bbox_max_y,bbox_max_z,dim_x,dim_y,dim_z,plane_height_m,centroid_height_m,"
                 "inplane_length_m,inplane_width_m,rule_pass,selected\n";
  cluster_csv.setf(std::ios::fmtflags(0), std::ios::floatfield);
  cluster_csv.precision(17);
  for (const auto& cluster : clusters) {
    const Eigen::Vector3d dims = cluster.bbox_max - cluster.bbox_min;
    cluster_csv << cluster.id << "," << cluster.points.size() << "," << cluster.centroid.x() << ","
                << cluster.centroid.y() << "," << cluster.centroid.z() << "," << cluster.bbox_min.x() << ","
                << cluster.bbox_min.y() << "," << cluster.bbox_min.z() << "," << cluster.bbox_max.x() << ","
                << cluster.bbox_max.y() << "," << cluster.bbox_max.z() << "," << dims.x() << "," << dims.y()
                << "," << dims.z() << "," << cluster.height_peak << "," << cluster.centroid_height << ","
                << cluster.length_m << "," << cluster.width_m << "," << (cluster.rule_pass ? 1 : 0) << ","
                << (cluster.selected ? 1 : 0) << "\n";
  }

  std::ofstream preprocess(out_dir + "/preprocessing_metrics.json");
  preprocess << "{\n";
  preprocess << "  \"input_points\": " << raw.points.size() << ",\n";
  preprocess << "  \"voxel_leaf_size_m\": " << jsonNumber(kVoxelLeaf) << ",\n";
  preprocess << "  \"voxel_index_fallback\": " << (voxel_fallback ? "true" : "false") << ",\n";
  preprocess << "  \"voxel_points\": " << voxel.size() << ",\n";
  preprocess << "  \"voxel_reduction_ratio\": " << jsonNumber(1.0 - static_cast<double>(voxel.size()) / static_cast<double>(raw.points.size())) << ",\n";
  preprocess << "  \"sor_mean_k\": " << kSorMeanK << ",\n";
  preprocess << "  \"sor_stddev\": " << jsonNumber(kSorStddev) << ",\n";
  preprocess << "  \"sor_input_points\": " << voxel.size() << ",\n";
  preprocess << "  \"sor_output_points\": " << denoised.size() << ",\n";
  preprocess << "  \"sor_removed_points\": " << (voxel.size() - denoised.size()) << ",\n";
  preprocess << "  \"ransac_distance_m\": " << jsonNumber(kRansacDistance) << ",\n";
  preprocess << "  \"ransac_iterations\": " << kRansacIterations << ",\n";
  preprocess << "  \"ransac_seed\": " << kRansacSeed << ",\n";
  preprocess << "  \"plane_a\": " << jsonNumber(plane.normal.x()) << ",\n";
  preprocess << "  \"plane_b\": " << jsonNumber(plane.normal.y()) << ",\n";
  preprocess << "  \"plane_c\": " << jsonNumber(plane.normal.z()) << ",\n";
  preprocess << "  \"plane_d\": " << jsonNumber(plane.d) << ",\n";
  preprocess << "  \"plane_inliers\": " << inliers.size() << ",\n";
  preprocess << "  \"plane_inlier_ratio\": " << jsonNumber(static_cast<double>(inliers.size()) / static_cast<double>(denoised.size())) << ",\n";
  preprocess << "  \"plane_remaining\": " << without_plane.size() << ",\n";
  preprocess << "  \"above_min_height_points\": " << above.size() << ",\n";
  preprocess << "  \"min_height_m\": " << jsonNumber(kMinHeight) << ",\n";
  preprocess << "  \"cluster_count\": " << clusters.size() << ",\n";
  preprocess << "  \"shoe_selected\": " << (selected ? "true" : "false") << "\n";
  preprocess << "}\n";
  if (!selected) {
    std::cerr << "신발 규칙을 통과한 cluster가 없습니다.\n";
    return 4;
  }
  writeCloud(out_dir + "/stage_shoe.bin", selected->points);
  const Frame frame = pcaFrame(selected->points);
  std::ofstream pca_json(out_dir + "/pca_frame.json");
  pca_json << "{\n";
  pca_json << "  \"centroid\": [" << jsonNumber(frame.centroid.x()) << ", " << jsonNumber(frame.centroid.y()) << ", " << jsonNumber(frame.centroid.z()) << "],\n";
  pca_json << "  \"eigenvalues\": [" << jsonNumber(frame.eigenvalues.x()) << ", " << jsonNumber(frame.eigenvalues.y()) << ", " << jsonNumber(frame.eigenvalues.z()) << "],\n";
  pca_json << "  \"axis_0\": [" << jsonNumber(frame.axis0.x()) << ", " << jsonNumber(frame.axis0.y()) << ", " << jsonNumber(frame.axis0.z()) << "],\n";
  pca_json << "  \"axis_1\": [" << jsonNumber(frame.axis1.x()) << ", " << jsonNumber(frame.axis1.y()) << ", " << jsonNumber(frame.axis1.z()) << "],\n";
  pca_json << "  \"axis_2\": [" << jsonNumber(frame.axis2.x()) << ", " << jsonNumber(frame.axis2.y()) << ", " << jsonNumber(frame.axis2.z()) << "]\n";
  pca_json << "}\n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr projected(new pcl::PointCloud<pcl::PointXYZ>);
  projected->reserve(selected->points.size());
  std::ofstream projected_csv(out_dir + "/projected_points.csv");
  projected_csv << "u,v,x,y,z\n";
  projected_csv.setf(std::ios::fmtflags(0), std::ios::floatfield);
  projected_csv.precision(17);
  double u_min = std::numeric_limits<double>::infinity();
  double u_max = -u_min;
  double v_min = u_min;
  double v_max = u_max;
  for (const auto& point : selected->points) {
    const Eigen::Vector3d delta = point - frame.centroid;
    const double u = frame.axis0.dot(delta);
    const double v = frame.axis1.dot(delta);
    u_min = std::min(u_min, u);
    u_max = std::max(u_max, u);
    v_min = std::min(v_min, v);
    v_max = std::max(v_max, v);
    projected->push_back(pcl::PointXYZ(static_cast<float>(u), static_cast<float>(v), 0.f));
    projected_csv << u << "," << v << "," << point.x() << "," << point.y() << "," << point.z() << "\n";
  }
  projected->width = static_cast<std::uint32_t>(projected->size());
  projected->height = 1;
  projected->is_dense = true;

  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(projected);
  std::vector<float> neighbor_distance;
  neighbor_distance.reserve(projected->size());
  std::vector<int> index(2);
  std::vector<float> squared(2);
  for (const auto& point : projected->points) {
    if (tree.nearestKSearch(point, 2, index, squared) == 2) {
      neighbor_distance.push_back(std::sqrt(squared[1]));
    }
  }
  std::sort(neighbor_distance.begin(), neighbor_distance.end());
  auto percentile = [&](double rank) {
    const double position = rank * static_cast<double>(neighbor_distance.size() - 1);
    const std::size_t lower = static_cast<std::size_t>(std::floor(position));
    const std::size_t upper = std::min(lower + 1, neighbor_distance.size() - 1);
    const double weight = position - static_cast<double>(lower);
    return (1.0 - weight) * neighbor_distance[lower] + weight * neighbor_distance[upper];
  };
  const double nn_median = percentile(0.50);
  const double nn_p90 = percentile(0.90);
  const double nn_p95 = percentile(0.95);
  const double spacing_scale = std::max(nn_p95, static_cast<double>(kVoxelLeaf));

  pcl::ConvexHull<pcl::PointXYZ> convex;
  convex.setDimension(2);
  convex.setInputCloud(projected);
  pcl::PointCloud<pcl::PointXYZ> convex_cloud;
  std::vector<pcl::Vertices> convex_polygons;
  convex.reconstruct(convex_cloud, convex_polygons);
  std::vector<Eigen::Vector2d> convex_polygon;
  if (!convex_polygons.empty()) {
    for (const auto index : convex_polygons[0].vertices) {
      convex_polygon.emplace_back(convex_cloud[index].x, convex_cloud[index].y);
    }
  }
  const double convex_area = std::abs(shoelace(convex_polygon));
  const double convex_perimeter = perimeterOf(convex_polygon);

  const double factors[] = {2.0, 3.0, 4.0, 6.0, 8.0, 12.0};
  std::vector<Hull> candidates;
  for (const double factor : factors) {
    candidates.push_back(makeHull(projected, factor * spacing_scale, convex_area, convex_perimeter));
  }
  int chosen = -1;
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    const Hull& hull = candidates[index];
    if (!hull.valid) {
      continue;
    }
    if (hull.area_ratio < 0.55 || hull.perimeter_ratio > 3.0) {
      continue;
    }
    if (chosen < 0 || hull.alpha < candidates[static_cast<std::size_t>(chosen)].alpha) {
      chosen = static_cast<int>(index);
    }
  }
  std::string alpha_rule = "smallest_topology_ok_area_ratio_ge_0.55_perimeter_ratio_le_3";
  if (chosen < 0) {
    alpha_rule = "none";
  }
  std::ofstream sampling(out_dir + "/sampling_scale.json");
  sampling << "{\n";
  sampling << "  \"nn_median_m\": " << jsonNumber(nn_median) << ",\n";
  sampling << "  \"nn_p90_m\": " << jsonNumber(nn_p90) << ",\n";
  sampling << "  \"nn_p95_m\": " << jsonNumber(nn_p95) << ",\n";
  sampling << "  \"spacing_scale_m\": " << jsonNumber(spacing_scale) << ",\n";
  sampling << "  \"u_min\": " << jsonNumber(u_min) << ",\n";
  sampling << "  \"u_max\": " << jsonNumber(u_max) << ",\n";
  sampling << "  \"v_min\": " << jsonNumber(v_min) << ",\n";
  sampling << "  \"v_max\": " << jsonNumber(v_max) << ",\n";
  sampling << "  \"projected_point_count\": " << projected->size() << ",\n";
  sampling << "  \"convex_area_m2\": " << jsonNumber(convex_area) << ",\n";
  sampling << "  \"convex_perimeter_m\": " << jsonNumber(convex_perimeter) << "\n";
  sampling << "}\n";
  std::ofstream alpha_json(out_dir + "/alpha_candidates.json");
  alpha_json << "{\n  \"rule\": \"" << alpha_rule << "\",\n  \"candidates\": [\n";
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    const Hull& hull = candidates[index];
    alpha_json << "    {\"alpha_m\": " << jsonNumber(hull.alpha) << ", \"loops\": " << hull.loops
               << ", \"intersections\": " << hull.intersections << ", \"vertices\": " << hull.polygon.size()
               << ", \"area_m2\": " << jsonNumber(hull.area) << ", \"perimeter_m\": " << jsonNumber(hull.perimeter)
               << ", \"area_ratio\": " << jsonNumber(hull.area_ratio)
               << ", \"perimeter_ratio\": " << jsonNumber(hull.perimeter_ratio)
               << ", \"valid\": " << (hull.valid ? "true" : "false") << ", \"reason\": \"" << hull.reason
               << "\", \"selected\": " << (static_cast<int>(index) == chosen ? "true" : "false") << "}";
    alpha_json << (index + 1 == candidates.size() ? "\n" : ",\n");
  }
  alpha_json << "  ]\n}\n";
  if (chosen < 0) {
    std::cerr << "topology를 만족하는 concave hull이 없습니다.\n";
    return 5;
  }
  const Hull& hull = candidates[static_cast<std::size_t>(chosen)];
  pcl::KdTreeFLANN<pcl::PointXYZ> shoe_tree;
  auto shoe_cloud = toPcl(selected->points);
  shoe_tree.setInputCloud(shoe_cloud);
  std::ofstream boundary_2d(out_dir + "/boundary_2d.csv");
  std::ofstream boundary_3d(out_dir + "/boundary_3d.csv");
  std::ofstream support_csv(out_dir + "/boundary_support.csv");
  std::ofstream spacing_csv(out_dir + "/boundary_spacing.csv");
  boundary_2d << "index,u,v\n";
  boundary_3d << "index,x,y,z\n";
  support_csv << "index,distance_m\n";
  spacing_csv << "index,distance_m\n";
  boundary_2d.setf(std::ios::fmtflags(0), std::ios::floatfield);
  boundary_3d.setf(std::ios::fmtflags(0), std::ios::floatfield);
  support_csv.setf(std::ios::fmtflags(0), std::ios::floatfield);
  spacing_csv.setf(std::ios::fmtflags(0), std::ios::floatfield);
  boundary_2d.precision(17);
  boundary_3d.precision(17);
  support_csv.precision(17);
  spacing_csv.precision(17);
  std::vector<Eigen::Vector3d> boundary_points;
  std::vector<double> support;
  boundary_points.reserve(hull.polygon.size());
  for (std::size_t index = 0; index < hull.polygon.size(); ++index) {
    const Eigen::Vector2d& uv = hull.polygon[index];
    const Eigen::Vector3d plane_point = frame.centroid + uv.x() * frame.axis0 + uv.y() * frame.axis1;
    pcl::PointXYZ query(
        static_cast<float>(plane_point.x()), static_cast<float>(plane_point.y()), static_cast<float>(plane_point.z()));
    std::vector<int> neighbor(1);
    std::vector<float> neighbor_squared(1);
    shoe_tree.nearestKSearch(query, 1, neighbor, neighbor_squared);
    const Eigen::Vector3d surface = selected->points[static_cast<std::size_t>(neighbor[0])];
    const double distance = std::sqrt(neighbor_squared[0]);
    boundary_points.push_back(surface);
    support.push_back(distance);
    boundary_2d << index << "," << uv.x() << "," << uv.y() << "\n";
    boundary_3d << index << "," << surface.x() << "," << surface.y() << "," << surface.z() << "\n";
    support_csv << index << "," << distance << "\n";
  }
  int worst = 0;
  double worst_distance = -1.0;
  for (std::size_t index = 0; index < boundary_points.size(); ++index) {
    const double distance = (boundary_points[(index + 1) % boundary_points.size()] - boundary_points[index]).norm();
    spacing_csv << index << "," << distance << "\n";
    if (distance > worst_distance) {
      worst_distance = distance;
      worst = static_cast<int>(index);
    }
  }
  auto summarize = [](const std::vector<double>& values) {
    std::vector<double> sorted = values;
    std::sort(sorted.begin(), sorted.end());
    double sum = 0.0;
    for (const double value : sorted) {
      sum += value;
    }
    auto at = [&](double rank) {
      const double position = rank * static_cast<double>(sorted.size() - 1);
      const std::size_t lower = static_cast<std::size_t>(std::floor(position));
      const std::size_t upper = std::min(lower + 1, sorted.size() - 1);
      const double weight = position - static_cast<double>(lower);
      return (1.0 - weight) * sorted[lower] + weight * sorted[upper];
    };
    std::ostringstream stream;
    stream.setf(std::ios::fmtflags(0), std::ios::floatfield);
    stream.precision(17);
    stream << "\"mean\": " << (sum / static_cast<double>(sorted.size())) << ", \"median\": " << at(0.5)
           << ", \"p95\": " << at(0.95) << ", \"max\": " << sorted.back();
    return stream.str();
  };
  std::ofstream summary(out_dir + "/boundary_summary.json");
  summary << "{\n";
  summary << "  \"vertex_count\": " << hull.polygon.size() << ",\n";
  summary << "  \"loop_count\": " << hull.loops << ",\n";
  summary << "  \"closed\": true,\n";
  summary << "  \"ordered\": true,\n";
  summary << "  \"self_intersection_count\": " << hull.intersections << ",\n";
  summary << "  \"signed_area_m2\": " << jsonNumber(hull.area) << ",\n";
  summary << "  \"perimeter_m\": " << jsonNumber(hull.perimeter) << ",\n";
  summary << "  \"alpha_m\": " << jsonNumber(hull.alpha) << ",\n";
  summary << "  \"reprojection\": \"nearest_observed_surface_point\",\n";
  summary << "  \"support_is_lift_distance\": true,\n";
  summary << "  \"support\": {" << summarize(support) << "},\n";
  summary << "  \"worst_gap_index\": " << worst << ",\n";
  summary << "  \"worst_gap_m\": " << jsonNumber(worst_distance) << ",\n";
  summary << "  \"selected_cluster_id\": " << selected->id << ",\n";
  summary << "  \"selected_point_count\": " << selected->points.size() << "\n";
  summary << "}\n";
  std::cout << "selected_cluster=" << selected->id << " vertices=" << hull.polygon.size()
            << " alpha=" << hull.alpha << "\n";
  return 0;
}
