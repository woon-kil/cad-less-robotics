// P-008-R1GEO. capture_01 XYZ에서 지지 평면을 제거하고 군집을 만든다.
// RGB는 읽어서 버린다. 분할 특징으로 쓰지 않는다.
// R1H 파일과 path_extractor는 수정하지 않는다.

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct Params {
  int u0{300};
  int u1{760};
  int v0{300};
  int v1{620};
  double fx{610.5073852539062};
  double fy{610.349853515625};
  double cx{641.7918090820312};
  double cy{364.4433288574219};
  double z_min{0.2};
  double z_max{1.5};
  float voxel_leaf{0.002f};
  double ransac_distance{0.004};
  int ransac_iterations{200};
  int ransac_seed{1};
  double cluster_tolerance{0.01};
  int min_cluster_size{50};
  double max_object_height{0.18};
  double min_inlier_ratio{0.2};
  int min_inlier_count{1000};
};

bool forbiddenInput(const std::string& path) {
  return path.find("capture_02") != std::string::npos || path.find("capture_03") != std::string::npos ||
      path.find("capture_04") != std::string::npos || path.find("capture_05") != std::string::npos;
}

double jsonNumber(const std::string& text, const std::string& key, double fallback) {
  const std::string token = "\"" + key + "\"";
  const auto at = text.find(token);
  if (at == std::string::npos) {
    return fallback;
  }
  auto pos = text.find(':', at + token.size());
  if (pos == std::string::npos) {
    return fallback;
  }
  ++pos;
  while (pos < text.size() && (text[pos] == ' ' || text[pos] == '\n')) {
    ++pos;
  }
  if (pos < text.size() && text[pos] == '[') {
    ++pos;
  }
  return std::strtod(text.c_str() + pos, nullptr);
}

int jsonInt(const std::string& text, const std::string& key, int fallback) {
  return static_cast<int>(std::lround(jsonNumber(text, key, fallback)));
}

Params loadParams(const std::string& path) {
  std::ifstream input(path);
  std::stringstream buffer;
  buffer << input.rdbuf();
  const std::string text = buffer.str();
  Params params;
  params.u0 = jsonInt(text, "pixel_u", params.u0);
  const auto pixel_u = text.find("\"pixel_u\"");
  if (pixel_u != std::string::npos) {
    const auto bracket = text.find('[', pixel_u);
    params.u0 = static_cast<int>(std::strtod(text.c_str() + bracket + 1, nullptr));
    const auto comma = text.find(',', bracket);
    params.u1 = static_cast<int>(std::strtod(text.c_str() + comma + 1, nullptr));
  }
  const auto pixel_v = text.find("\"pixel_v\"");
  if (pixel_v != std::string::npos) {
    const auto bracket = text.find('[', pixel_v);
    params.v0 = static_cast<int>(std::strtod(text.c_str() + bracket + 1, nullptr));
    const auto comma = text.find(',', bracket);
    params.v1 = static_cast<int>(std::strtod(text.c_str() + comma + 1, nullptr));
  }
  params.fx = jsonNumber(text, "fx", params.fx);
  params.fy = jsonNumber(text, "fy", params.fy);
  params.cx = jsonNumber(text, "cx", params.cx);
  params.cy = jsonNumber(text, "cy", params.cy);
  params.z_min = jsonNumber(text, "z_min", params.z_min);
  params.z_max = jsonNumber(text, "z_max", params.z_max);
  params.voxel_leaf = static_cast<float>(jsonNumber(text, "voxel_leaf", params.voxel_leaf));
  params.ransac_distance = jsonNumber(text, "ransac_distance", params.ransac_distance);
  params.ransac_iterations = jsonInt(text, "ransac_iterations", params.ransac_iterations);
  params.ransac_seed = jsonInt(text, "ransac_seed", params.ransac_seed);
  params.cluster_tolerance = jsonNumber(text, "cluster_tolerance", params.cluster_tolerance);
  params.min_cluster_size = jsonInt(text, "min_cluster_size", params.min_cluster_size);
  params.max_object_height = jsonNumber(text, "max_object_height", params.max_object_height);
  params.min_inlier_ratio = jsonNumber(text, "min_inlier_ratio", params.min_inlier_ratio);
  params.min_inlier_count = jsonInt(text, "min_inlier_count", params.min_inlier_count);
  return params;
}

std::string jsonFloat(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream out;
  out.setf(std::ios::fmtflags(0), std::ios::floatfield);
  out.precision(17);
  out << value;
  return out.str();
}

bool loadCloud(const std::string& path, std::vector<Eigen::Vector3d>& points, std::string& error) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    error = "PCD를 열지 못했습니다.";
    return false;
  }
  std::string line;
  int count = -1;
  bool binary = false;
  bool rgb = false;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.rfind("POINTS ", 0) == 0) {
      count = std::stoi(line.substr(7));
    }
    if (line.rfind("FIELDS ", 0) == 0 && line.find("rgb") != std::string::npos) {
      rgb = true;
    }
    if (line == "DATA binary") {
      binary = true;
      break;
    }
  }
  if (!binary || count < 0 || !rgb) {
    error = "x y z rgb binary PCD가 아닙니다.";
    return false;
  }
  points.resize(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index) {
    float values[4];
    input.read(reinterpret_cast<char*>(values), sizeof(values));
    if (!input) {
      error = "PCD 본문이 잘렸습니다.";
      return false;
    }
    points[static_cast<std::size_t>(index)] = Eigen::Vector3d(values[0], values[1], values[2]);
  }
  return true;
}

bool writeXyz(const std::filesystem::path& path, const std::vector<Eigen::Vector3d>& points) {
  std::ofstream output(path, std::ios::binary);
  if (!output) {
    return false;
  }
  output << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n";
  output << "WIDTH " << points.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n";
  output << "POINTS " << points.size() << "\nDATA binary\n";
  for (const auto& point : points) {
    const float values[3] = {static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z())};
    output.write(reinterpret_cast<const char*>(values), sizeof(values));
  }
  return static_cast<bool>(output);
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
  cloud->is_dense = false;
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

double percentile(std::vector<double> values, double rank) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const double position = rank * static_cast<double>(values.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = std::min(lower + 1, values.size() - 1);
  const double weight = position - static_cast<double>(lower);
  return (1.0 - weight) * values[lower] + weight * values[upper];
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "사용법: p008r1geo <cloud.pcd> <parameters.json> <out_dir>\n";
    return 2;
  }
  const std::string cloud_path = argv[1];
  const std::string param_path = argv[2];
  const std::filesystem::path out_dir = argv[3];
  if (forbiddenInput(cloud_path)) {
    std::cerr << "holdout capture는 열지 않습니다.\n";
    return 2;
  }
  std::string error;
  std::vector<Eigen::Vector3d> raw;
  if (!loadCloud(cloud_path, raw, error)) {
    std::cerr << error << "\n";
    return 2;
  }
  const Params params = loadParams(param_path);
  std::vector<Eigen::Vector3d> workspace;
  workspace.reserve(raw.size() / 4);
  for (const auto& point : raw) {
    if (!point.allFinite() || point.z() < params.z_min || point.z() > params.z_max || point.z() <= 0.0) {
      continue;
    }
    const double u = params.fx * point.x() / point.z() + params.cx;
    const double v = params.fy * point.y() / point.z() + params.cy;
    if (u < params.u0 || u >= params.u1 || v < params.v0 || v >= params.v1) {
      continue;
    }
    workspace.push_back(point);
  }
  std::filesystem::create_directories(out_dir);
  if (!writeXyz(out_dir / "01_input.pcd", workspace)) {
    std::cerr << "01_input.pcd를 쓰지 못했습니다.\n";
    return 2;
  }

  auto input = toPcl(workspace);
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setInputCloud(input);
  grid.setLeafSize(params.voxel_leaf, params.voxel_leaf, params.voxel_leaf);
  grid.filter(*voxel);
  const auto voxel_points = fromPcl(*voxel);
  if (!writeXyz(out_dir / "02_workspace.pcd", voxel_points)) {
    std::cerr << "02_workspace.pcd를 쓰지 못했습니다.\n";
    return 2;
  }

  std::srand(static_cast<unsigned>(params.ransac_seed));
  pcl::SACSegmentation<pcl::PointXYZ> segmentation;
  pcl::PointIndices::Ptr inlier_indices(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(params.ransac_distance);
  segmentation.setMaxIterations(params.ransac_iterations);
  segmentation.setInputCloud(voxel);
  segmentation.segment(*inlier_indices, *coefficients);

  bool identifiable = false;
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  double plane_d = 0.0;
  bool flipped = false;
  double residual_median = std::numeric_limits<double>::quiet_NaN();
  double residual_p95 = std::numeric_limits<double>::quiet_NaN();
  const int inlier_count = inlier_indices ? static_cast<int>(inlier_indices->indices.size()) : 0;
  const double inlier_ratio = voxel->empty() ? 0.0 : static_cast<double>(inlier_count) / static_cast<double>(voxel->size());
  std::vector<Eigen::Vector3d> plane_points;
  std::vector<Eigen::Vector3d> object_points;
  std::vector<int> labels;

  if (coefficients && coefficients->values.size() >= 4 && inlier_count >= 3) {
    normal = Eigen::Vector3d(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    const double length = normal.norm();
    if (length >= 1e-9) {
      normal /= length;
      plane_d = coefficients->values[3] / length;
      Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
      std::vector<double> residuals;
      residuals.reserve(static_cast<std::size_t>(inlier_count));
      for (const int index : inlier_indices->indices) {
        const auto& point = voxel->points[static_cast<std::size_t>(index)];
        centroid += Eigen::Vector3d(point.x, point.y, point.z);
      }
      centroid /= static_cast<double>(inlier_count);
      const Eigen::Vector3d toward_camera = -centroid;
      if (toward_camera.dot(normal) < 0.0) {
        normal = -normal;
        plane_d = -plane_d;
        flipped = true;
      }
      for (const int index : inlier_indices->indices) {
        const auto& point = voxel->points[static_cast<std::size_t>(index)];
        const Eigen::Vector3d xyz(point.x, point.y, point.z);
        plane_points.push_back(xyz);
        residuals.push_back(std::abs(normal.dot(xyz) + plane_d));
      }
      residual_median = percentile(residuals, 0.5);
      residual_p95 = percentile(residuals, 0.95);
      identifiable = inlier_count >= params.min_inlier_count && inlier_ratio >= params.min_inlier_ratio &&
          residual_median <= params.ransac_distance;
    }
  }
  if (!writeXyz(out_dir / "03_support_plane.pcd", plane_points)) {
    std::cerr << "03_support_plane.pcd를 쓰지 못했습니다.\n";
    return 2;
  }

  struct Cluster {
    int id{0};
    std::vector<Eigen::Vector3d> points;
    Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
    Eigen::Vector3d aabb_min{Eigen::Vector3d::Zero()};
    Eigen::Vector3d aabb_max{Eigen::Vector3d::Zero()};
    double height_min{0};
    double height_median{0};
    double height_p95{0};
    double height_max{0};
  };
  std::vector<Cluster> clusters;
  if (identifiable) {
    pcl::PointIndices::Ptr keep(new pcl::PointIndices);
    for (int index = 0; index < static_cast<int>(voxel->size()); ++index) {
      const auto& point = voxel->points[static_cast<std::size_t>(index)];
      const double height = normal.dot(Eigen::Vector3d(point.x, point.y, point.z)) + plane_d;
      if (height > params.ransac_distance && height <= params.max_object_height) {
        keep->indices.push_back(index);
      }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(voxel);
    extract.setIndices(keep);
    extract.setNegative(false);
    extract.filter(*objects);
    object_points = fromPcl(*objects);

    if (!objects->empty()) {
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(objects);
      std::vector<pcl::PointIndices> groups;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
      clustering.setClusterTolerance(params.cluster_tolerance);
      clustering.setMinClusterSize(params.min_cluster_size);
      clustering.setMaxClusterSize(static_cast<int>(objects->size()));
      clustering.setSearchMethod(tree);
      clustering.setInputCloud(objects);
      clustering.extract(groups);
      labels.assign(objects->size(), -1);
      int next_label = 0;
      for (const auto& group : groups) {
        Cluster cluster;
        cluster.id = next_label;
        cluster.points.reserve(group.indices.size());
        std::vector<double> heights;
        heights.reserve(group.indices.size());
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        for (const int local : group.indices) {
          labels[static_cast<std::size_t>(local)] = next_label;
          const auto& point = objects->points[static_cast<std::size_t>(local)];
          const Eigen::Vector3d xyz(point.x, point.y, point.z);
          cluster.points.push_back(xyz);
          sum += xyz;
          heights.push_back(normal.dot(xyz) + plane_d);
        }
        ++next_label;
        cluster.centroid = sum / static_cast<double>(cluster.points.size());
        cluster.aabb_min = cluster.points.front();
        cluster.aabb_max = cluster.points.front();
        for (const auto& point : cluster.points) {
          cluster.aabb_min = cluster.aabb_min.cwiseMin(point);
          cluster.aabb_max = cluster.aabb_max.cwiseMax(point);
        }
        cluster.height_min = percentile(heights, 0.0);
        cluster.height_median = percentile(heights, 0.5);
        cluster.height_p95 = percentile(heights, 0.95);
        cluster.height_max = percentile(heights, 1.0);
        clusters.push_back(std::move(cluster));
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
      std::vector<int> remap(static_cast<std::size_t>(next_label), -1);
      for (std::size_t id = 0; id < clusters.size(); ++id) {
        remap[static_cast<std::size_t>(clusters[id].id)] = static_cast<int>(id);
        clusters[id].id = static_cast<int>(id);
      }
      for (int& label : labels) {
        if (label >= 0) {
          label = remap[static_cast<std::size_t>(label)];
        }
      }
    }
  }
  if (!writeXyz(out_dir / "04_non_plane.pcd", object_points)) {
    std::cerr << "04_non_plane.pcd를 쓰지 못했습니다.\n";
    return 2;
  }
  {
    std::ofstream labels_out(out_dir / "04_non_plane_labels.bin", std::ios::binary);
    const std::int32_t count = static_cast<std::int32_t>(labels.size());
    labels_out.write(reinterpret_cast<const char*>(&count), sizeof(count));
    if (!labels.empty()) {
      labels_out.write(reinterpret_cast<const char*>(labels.data()), static_cast<std::streamsize>(labels.size() * sizeof(int)));
    }
  }

  {
    std::ofstream plane_out(out_dir / "support_plane.json");
    plane_out << "{\n";
    plane_out << "  \"identifiable\": " << (identifiable ? "true" : "false") << ",\n";
    plane_out << "  \"coefficients\": [" << jsonFloat(normal.x()) << ", " << jsonFloat(normal.y()) << ", "
              << jsonFloat(normal.z()) << ", " << jsonFloat(plane_d) << "],\n";
    plane_out << "  \"normal\": [" << jsonFloat(normal.x()) << ", " << jsonFloat(normal.y()) << ", " << jsonFloat(normal.z())
              << "],\n";
    plane_out << "  \"flipped_once\": " << (flipped ? "true" : "false") << ",\n";
    plane_out << "  \"ransac_distance\": " << jsonFloat(params.ransac_distance) << ",\n";
    plane_out << "  \"ransac_iterations\": " << params.ransac_iterations << ",\n";
    plane_out << "  \"inlier_count\": " << inlier_count << ",\n";
    plane_out << "  \"workspace_voxel_count\": " << voxel->size() << ",\n";
    plane_out << "  \"inlier_ratio\": " << jsonFloat(inlier_ratio) << ",\n";
    plane_out << "  \"residual_median\": " << jsonFloat(residual_median) << ",\n";
    plane_out << "  \"residual_p95\": " << jsonFloat(residual_p95) << ",\n";
    plane_out << "  \"workspace_count\": " << workspace.size() << "\n";
    plane_out << "}\n";
  }
  {
    std::ofstream cluster_out(out_dir / "clusters.json");
    cluster_out << "[\n";
    for (std::size_t index = 0; index < clusters.size(); ++index) {
      const auto& cluster = clusters[index];
      const Eigen::Vector3d extent = cluster.aabb_max - cluster.aabb_min;
      cluster_out << "  {\n";
      cluster_out << "    \"id\": " << cluster.id << ",\n";
      cluster_out << "    \"point_count\": " << cluster.points.size() << ",\n";
      cluster_out << "    \"centroid\": [" << jsonFloat(cluster.centroid.x()) << ", " << jsonFloat(cluster.centroid.y())
                  << ", " << jsonFloat(cluster.centroid.z()) << "],\n";
      cluster_out << "    \"aabb_min\": [" << jsonFloat(cluster.aabb_min.x()) << ", " << jsonFloat(cluster.aabb_min.y())
                  << ", " << jsonFloat(cluster.aabb_min.z()) << "],\n";
      cluster_out << "    \"aabb_max\": [" << jsonFloat(cluster.aabb_max.x()) << ", " << jsonFloat(cluster.aabb_max.y())
                  << ", " << jsonFloat(cluster.aabb_max.z()) << "],\n";
      cluster_out << "    \"extent\": [" << jsonFloat(extent.x()) << ", " << jsonFloat(extent.y()) << ", "
                  << jsonFloat(extent.z()) << "],\n";
      cluster_out << "    \"height_min\": " << jsonFloat(cluster.height_min) << ",\n";
      cluster_out << "    \"height_median\": " << jsonFloat(cluster.height_median) << ",\n";
      cluster_out << "    \"height_p95\": " << jsonFloat(cluster.height_p95) << ",\n";
      cluster_out << "    \"height_max\": " << jsonFloat(cluster.height_max) << "\n";
      cluster_out << "  }" << (index + 1 == clusters.size() ? "\n" : ",\n");
    }
    cluster_out << "]\n";
  }
  std::cout << "workspace " << workspace.size() << " voxel " << voxel->size() << " inliers " << inlier_count
            << " ratio " << inlier_ratio << " identifiable " << identifiable << " clusters " << clusters.size()
            << " objects " << object_points.size() << "\n";
  return 0;
}
