// P-008-R1F. prism cloud의 L2 연결만 진단한다.
// cluster tolerance를 고르지 않고, prism·height·voxel·SOR·RANSAC은 바꾸지 않는다.
// t_internal: 추적점 component가 진단 seed 전체를 담는 가장 작은 tolerance.
// t_env: 그 component 점 수가 2 * |S|를 넘는 가장 작은 tolerance.
// 둘 다 전체 prism radius graph다. seed induced MST는 보조 수치다.

#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

constexpr double kSearchMin = 0.001;
constexpr double kSearchMax = 0.050;
constexpr double kSearchStep = 0.0001;
constexpr int kMarginSteps = 20;
constexpr double kTenMm = 0.01;
constexpr int kMinClusterSize = 50;
constexpr int kExpectedSelected = 76248;
constexpr int kExpectedSeed = 2861;
constexpr int kExpectedComponent = 23734;
constexpr int kExpectedSeedInComponent = 2505;

const double kSweep[] = {0.002, 0.003, 0.004, 0.005, 0.006, 0.007, 0.008, 0.010, 0.012, 0.015, 0.020, 0.030};

struct Vec3 {
  float x;
  float y;
  float z;
};

struct Key {
  std::uint32_t x;
  std::uint32_t y;
  std::uint32_t z;
  bool operator==(const Key& other) const { return x == other.x && y == other.y && z == other.z; }
};

struct KeyHash {
  std::size_t operator()(const Key& key) const {
    return (static_cast<std::size_t>(key.x) * 1315423911u) ^ (static_cast<std::size_t>(key.y) << 1) ^ key.z;
  }
};

struct Snapshot {
  int component_count = 0;
  int cluster_count = 0;
  int tracked_size = 0;
  int tracked_seeds = 0;
  int seed_component_count = 0;
  bool environment_merged = false;
  Vec3 bbox_min{};
  Vec3 bbox_max{};
};

struct Edge {
  double length = 0.0;
  int shoe = -1;
  int environment = -1;
};

class Dsu {
 public:
  explicit Dsu(int count) : parent_(count), size_(count, 1) {
    std::iota(parent_.begin(), parent_.end(), 0);
  }

  int find(int index) const {
    int current = index;
    while (parent_[static_cast<std::size_t>(current)] != current) {
      current = parent_[static_cast<std::size_t>(current)];
    }
    int cursor = index;
    while (parent_[static_cast<std::size_t>(cursor)] != current) {
      const int next = parent_[static_cast<std::size_t>(cursor)];
      parent_[static_cast<std::size_t>(cursor)] = current;
      cursor = next;
    }
    return current;
  }

  int unite(int left, int right) {
    left = find(left);
    right = find(right);
    if (left == right) {
      return left;
    }
    if (size_[left] < size_[right]) {
      std::swap(left, right);
    }
    parent_[right] = left;
    size_[left] += size_[right];
    return left;
  }

  int size(int index) const { return size_[static_cast<std::size_t>(find(index))]; }

 private:
  mutable std::vector<int> parent_;
  std::vector<int> size_;
};

bool forbiddenPath(const std::string& path) {
  return path.find("capture_02") != std::string::npos || path.find("capture_03") != std::string::npos ||
      path.find("capture_04") != std::string::npos || path.find("capture_05") != std::string::npos;
}

std::string jsonNumber(double value) {
  std::ostringstream stream;
  stream.setf(std::ios::fmtflags(0), std::ios::floatfield);
  stream.precision(17);
  stream << value;
  return stream.str();
}

Key keyOf(const Vec3& point) {
  Key key{};
  static_assert(sizeof(float) == sizeof(std::uint32_t), "float bits");
  std::memcpy(&key.x, &point.x, sizeof(float));
  std::memcpy(&key.y, &point.y, sizeof(float));
  std::memcpy(&key.z, &point.z, sizeof(float));
  return key;
}

double distanceBetween(const Vec3& left, const Vec3& right) {
  const double dx = static_cast<double>(left.x) - right.x;
  const double dy = static_cast<double>(left.y) - right.y;
  const double dz = static_cast<double>(left.z) - right.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

int searchKey(double tolerance) {
  return static_cast<int>(std::lround(tolerance / kSearchStep));
}

double toleranceAt(int key) { return static_cast<double>(key) * kSearchStep; }

bool loadBin(const std::string& path, std::vector<Vec3>& points, std::string& error) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    error = "bin을 열지 못했습니다: " + path;
    return false;
  }
  std::uint32_t count = 0;
  input.read(reinterpret_cast<char*>(&count), sizeof(count));
  points.resize(count);
  if (count == 0) {
    return true;
  }
  input.read(reinterpret_cast<char*>(points.data()), static_cast<std::streamsize>(count * sizeof(Vec3)));
  if (!input) {
    error = "bin 본문이 잘렸습니다: " + path;
    return false;
  }
  return true;
}

void writeBin(const std::string& path, const std::vector<Vec3>& points) {
  std::ofstream output(path, std::ios::binary);
  const auto count = static_cast<std::uint32_t>(points.size());
  output.write(reinterpret_cast<const char*>(&count), sizeof(count));
  if (!points.empty()) {
    output.write(reinterpret_cast<const char*>(points.data()), static_cast<std::streamsize>(points.size() * sizeof(Vec3)));
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr toCloud(const std::vector<Vec3>& points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->reserve(points.size());
  for (const auto& point : points) {
    cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

class RadiusGraph {
 public:
  RadiusGraph(const std::vector<Vec3>& points, const std::vector<char>& seed, int tracked)
      : points_(points), seed_(seed), tracked_(tracked), seed_count_(0) {
    for (const char flag : seed_) {
      seed_count_ += flag ? 1 : 0;
    }
    cloud_ = toCloud(points_);
    tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>);
    tree_->setInputCloud(cloud_);
  }

  int seedCount() const { return seed_count_; }
  int tracked() const { return tracked_; }
  const std::vector<Vec3>& points() const { return points_; }

  Snapshot snapshot(double tolerance) {
    const int key = searchKey(tolerance);
    const auto found = cache_.find(key);
    if (found != cache_.end() && std::abs(found->second.first - tolerance) < 1e-12) {
      return found->second.second;
    }
    const Snapshot value = measure(connect(tolerance));
    cache_[key] = {tolerance, value};
    std::cerr << "graph t=" << tolerance << " size=" << value.tracked_size << " seeds=" << value.tracked_seeds
              << "\n";
    return value;
  }

  Dsu connect(double tolerance) const {
    Dsu dsu(static_cast<int>(points_.size()));
    std::vector<int> neighbors;
    std::vector<float> squared;
    for (int index = 0; index < static_cast<int>(points_.size()); ++index) {
      neighbors.clear();
      squared.clear();
      tree_->radiusSearch(index, tolerance, neighbors, squared);
      for (const int neighbor : neighbors) {
        if (neighbor > index) {
          dsu.unite(index, neighbor);
        }
      }
    }
    return dsu;
  }

  Snapshot measure(const Dsu& dsu) const {
    Snapshot snapshot;
    std::vector<int> seen(points_.size(), 0);
    std::vector<int> seed_seen(points_.size(), 0);
    const int tracked_root = dsu.find(tracked_);
    bool bbox = false;
    for (int index = 0; index < static_cast<int>(points_.size()); ++index) {
      const int root = dsu.find(index);
      if (!seen[root]) {
        seen[root] = 1;
        ++snapshot.component_count;
        if (dsu.size(root) >= kMinClusterSize) {
          ++snapshot.cluster_count;
        }
      }
      if (seed_[static_cast<std::size_t>(index)] && !seed_seen[root]) {
        seed_seen[root] = 1;
        ++snapshot.seed_component_count;
      }
      if (root == tracked_root) {
        ++snapshot.tracked_size;
        snapshot.tracked_seeds += seed_[static_cast<std::size_t>(index)] ? 1 : 0;
        const Vec3& point = points_[static_cast<std::size_t>(index)];
        if (!bbox) {
          snapshot.bbox_min = point;
          snapshot.bbox_max = point;
          bbox = true;
        } else {
          snapshot.bbox_min.x = std::min(snapshot.bbox_min.x, point.x);
          snapshot.bbox_min.y = std::min(snapshot.bbox_min.y, point.y);
          snapshot.bbox_min.z = std::min(snapshot.bbox_min.z, point.z);
          snapshot.bbox_max.x = std::max(snapshot.bbox_max.x, point.x);
          snapshot.bbox_max.y = std::max(snapshot.bbox_max.y, point.y);
          snapshot.bbox_max.z = std::max(snapshot.bbox_max.z, point.z);
        }
      }
    }
    snapshot.environment_merged = snapshot.tracked_size > 2 * seed_count_;
    return snapshot;
  }

  bool allSeeds(const Snapshot& snapshot) const { return snapshot.tracked_seeds == seed_count_; }

  Edge bridge(double below, double tolerance) const {
    Dsu dsu = connect(below);
    const int threshold = 2 * seed_count_;
    std::vector<std::tuple<double, int, int>> shell;
    std::vector<int> neighbors;
    std::vector<float> squared;
    for (int index = 0; index < static_cast<int>(points_.size()); ++index) {
      neighbors.clear();
      squared.clear();
      tree_->radiusSearch(index, tolerance, neighbors, squared);
      for (const int neighbor : neighbors) {
        if (neighbor <= index) {
          continue;
        }
        if (dsu.find(index) == dsu.find(neighbor)) {
          continue;
        }
        shell.emplace_back(distanceBetween(points_[static_cast<std::size_t>(index)], points_[static_cast<std::size_t>(neighbor)]), index, neighbor);
      }
    }
    std::sort(shell.begin(), shell.end());
    Edge edge;
    for (const auto& item : shell) {
      const int left = std::get<1>(item);
      const int right = std::get<2>(item);
      const int left_root = dsu.find(left);
      const int right_root = dsu.find(right);
      const int tracked_now = dsu.find(tracked_);
      const bool left_tracked = left_root == tracked_now;
      const bool right_tracked = right_root == tracked_now;
      const int size_before = dsu.size(tracked_);
      dsu.unite(left, right);
      const int size_after = dsu.size(tracked_);
      if (size_before <= threshold && size_after > threshold && (left_tracked || right_tracked)) {
        edge.length = std::get<0>(item);
        edge.shoe = left_tracked ? left : right;
        edge.environment = left_tracked ? right : left;
        break;
      }
    }
    return edge;
  }

  std::vector<int> pathTo(double tolerance, int source, int target) const {
    std::vector<int> parent(points_.size(), -1);
    std::vector<char> seen(points_.size(), 0);
    std::vector<int> queue;
    queue.push_back(source);
    seen[static_cast<std::size_t>(source)] = 1;
    std::vector<int> neighbors;
    std::vector<float> squared;
    for (std::size_t cursor = 0; cursor < queue.size(); ++cursor) {
      const int index = queue[cursor];
      if (index == target) {
        break;
      }
      neighbors.clear();
      squared.clear();
      tree_->radiusSearch(index, tolerance, neighbors, squared);
      for (const int neighbor : neighbors) {
        if (seen[static_cast<std::size_t>(neighbor)]) {
          continue;
        }
        seen[static_cast<std::size_t>(neighbor)] = 1;
        parent[static_cast<std::size_t>(neighbor)] = index;
        queue.push_back(neighbor);
      }
    }
    std::vector<int> path;
    if (source == target) {
      path.push_back(source);
      return path;
    }
    if (parent[static_cast<std::size_t>(target)] < 0) {
      return path;
    }
    for (int index = target; index >= 0; index = parent[static_cast<std::size_t>(index)]) {
      path.push_back(index);
      if (index == source) {
        break;
      }
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  std::vector<Vec3> componentPoints(const Dsu& dsu, int index) const {
    std::vector<Vec3> points;
    const int root = dsu.find(index);
    for (int cursor = 0; cursor < static_cast<int>(points_.size()); ++cursor) {
      if (dsu.find(cursor) == root) {
        points.push_back(points_[static_cast<std::size_t>(cursor)]);
      }
    }
    return points;
  }

 private:
  const std::vector<Vec3>& points_;
  const std::vector<char>& seed_;
  int tracked_;
  int seed_count_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
  std::unordered_map<int, std::pair<double, Snapshot>> cache_;
};

struct Critical {
  bool resolved = false;
  int internal_key = -1;
  int environment_key = -1;
  std::string separation;
};

Critical findCritical(RadiusGraph& graph) {
  Critical critical;
  const int min_key = searchKey(kSearchMin);
  const int max_key = searchKey(kSearchMax);
  const Snapshot at_min = graph.snapshot(toleranceAt(min_key));
  const Snapshot at_max = graph.snapshot(toleranceAt(max_key));
  if (graph.allSeeds(at_min) || !graph.allSeeds(at_max) || at_min.environment_merged || !at_max.environment_merged) {
    critical.separation = "UNRESOLVED";
    return critical;
  }
  auto first = [&](auto predicate) {
    int low = min_key;
    int high = max_key;
    while (high - low > 1) {
      const int mid = (low + high) / 2;
      if (predicate(graph.snapshot(toleranceAt(mid)))) {
        high = mid;
      } else {
        low = mid;
      }
    }
    return high;
  };
  critical.internal_key = first([&](const Snapshot& snapshot) { return graph.allSeeds(snapshot); });
  critical.environment_key = first([&](const Snapshot& snapshot) { return snapshot.environment_merged; });
  const int steps = critical.environment_key - critical.internal_key;
  if (steps >= kMarginSteps) {
    critical.separation = "FEASIBLE";
  } else if (steps > 0) {
    critical.separation = "MARGINAL";
  } else {
    critical.separation = "NOT_FEASIBLE";
  }
  critical.resolved = true;
  return critical;
}

double percentile(std::vector<double> values, double rank) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double position = rank * static_cast<double>(values.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = std::min(lower + 1, values.size() - 1);
  const double weight = position - static_cast<double>(lower);
  return (1.0 - weight) * values[lower] + weight * values[upper];
}

bool readCoefficient(const std::string& path, const std::string& name, double& value) {
  std::ifstream input(path);
  std::string line;
  const std::string prefix = "\"" + name + "\"";
  while (std::getline(input, line)) {
    const auto pos = line.find(prefix);
    if (pos == std::string::npos) {
      continue;
    }
    const auto colon = line.find(':', pos);
    if (colon == std::string::npos) {
      return false;
    }
    value = std::stod(line.substr(colon + 1));
    return true;
  }
  return false;
}

int countMatched(const std::vector<Vec3>& needles, const std::unordered_map<Key, int, KeyHash>& haystack) {
  int count = 0;
  for (const auto& point : needles) {
    if (haystack.find(keyOf(point)) != haystack.end()) {
      ++count;
    }
  }
  return count;
}

std::vector<Vec3> grid(int width, int height, double spacing, double x0, double y0) {
  std::vector<Vec3> points;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      points.push_back(Vec3{
          static_cast<float>(x0 + static_cast<double>(x) * spacing),
          static_cast<float>(y0 + static_cast<double>(y) * spacing), 0.f});
    }
  }
  return points;
}

int selfTest() {
  const double spacing = 0.003;
  auto run = [&](double gap, const std::string& expected) {
    auto points = grid(4, 4, spacing, 0.0, 0.0);
    const auto environment = grid(8, 8, spacing, 3.0 * spacing + gap, 0.0);
    points.insert(points.end(), environment.begin(), environment.end());
    std::vector<char> seed(points.size(), 0);
    std::fill(seed.begin(), seed.begin() + 16, 1);
    RadiusGraph graph(points, seed, 0);
    const Critical critical = findCritical(graph);
    if (critical.separation != expected) {
      std::cerr << "self-test 실패 gap=" << gap << " got=" << critical.separation << " expected=" << expected << "\n";
      return false;
    }
    return true;
  };
  if (!run(0.012, "FEASIBLE") || !run(spacing * 0.5, "NOT_FEASIBLE")) {
    return 5;
  }
  if (!forbiddenPath("evidence/P-008A/captures/capture_02/cloud.pcd") || forbiddenPath("capture_01")) {
    std::cerr << "holdout 경로 검사가 실패했습니다.\n";
    return 5;
  }
  std::cout << "SELF_TEST PASS\n";
  return 0;
}

std::string separationInterval(const Critical& critical) {
  if (!critical.resolved || critical.environment_key <= critical.internal_key) {
    return "NONE";
  }
  return "(" + jsonNumber(toleranceAt(critical.internal_key)) + ", " + jsonNumber(toleranceAt(critical.environment_key)) + ")";
}

int runAnalysis(const std::string& repro_dir, const std::string& out_dir) {
  if (forbiddenPath(repro_dir) || forbiddenPath(out_dir)) {
    std::cerr << "capture_02~05는 사용할 수 없습니다.\n";
    return 3;
  }
  std::string error;
  std::vector<Vec3> selected;
  std::vector<Vec3> disk;
  std::vector<Vec3> shoe_cluster;
  std::vector<Vec3> inliers;
  if (!loadBin(repro_dir + "/stage_prism_selected.bin", selected, error) ||
      !loadBin(repro_dir + "/stage_shoe_disk.bin", disk, error) ||
      !loadBin(repro_dir + "/stage_shoe_cluster.bin", shoe_cluster, error) ||
      !loadBin(repro_dir + "/stage_inliers.bin", inliers, error)) {
    std::cerr << error << "\n";
    return 4;
  }
  if (static_cast<int>(selected.size()) != kExpectedSelected || static_cast<int>(disk.size()) != kExpectedSeed ||
      static_cast<int>(shoe_cluster.size()) != kExpectedComponent) {
    std::cerr << "R1E 재현 점 수가 다릅니다. selected=" << selected.size() << " disk=" << disk.size()
              << " component=" << shoe_cluster.size() << "\n";
    return 4;
  }
  std::unordered_map<Key, int, KeyHash> shoe_keys;
  for (const auto& point : shoe_cluster) {
    shoe_keys.emplace(keyOf(point), 1);
  }
  if (countMatched(disk, shoe_keys) != kExpectedSeedInComponent) {
    std::cerr << "10 mm component의 원판 점 수가 2505가 아닙니다.\n";
    return 4;
  }
  std::unordered_map<Key, std::vector<int>, KeyHash> selected_keys;
  for (int index = 0; index < static_cast<int>(selected.size()); ++index) {
    selected_keys[keyOf(selected[static_cast<std::size_t>(index)])].push_back(index);
  }
  std::vector<char> seed(selected.size(), 0);
  std::vector<int> seed_indices;
  for (const auto& point : disk) {
    const auto found = selected_keys.find(keyOf(point));
    if (found == selected_keys.end() || found->second.empty()) {
      std::cerr << "원판 점이 prism 선택 cloud에 없습니다.\n";
      return 4;
    }
    const int index = found->second.back();
    found->second.pop_back();
    seed[static_cast<std::size_t>(index)] = 1;
    seed_indices.push_back(index);
  }
  if (static_cast<int>(seed_indices.size()) != kExpectedSeed) {
    std::cerr << "원판 대응 수가 다릅니다.\n";
    return 4;
  }
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_z = 0.0;
  for (const auto& point : inliers) {
    origin_x += point.x;
    origin_y += point.y;
    origin_z += point.z;
  }
  origin_x /= static_cast<double>(inliers.size());
  origin_y /= static_cast<double>(inliers.size());
  origin_z /= static_cast<double>(inliers.size());
  const Vec3 origin{
      static_cast<float>(origin_x), static_cast<float>(origin_y), static_cast<float>(origin_z)};
  int tracked = seed_indices.front();
  double best = distanceBetween(selected[static_cast<std::size_t>(tracked)], origin);
  for (const int index : seed_indices) {
    const double value = distanceBetween(selected[static_cast<std::size_t>(index)], origin);
    if (value < best) {
      best = value;
      tracked = index;
    }
  }
  RadiusGraph graph(selected, seed, tracked);
  const Snapshot ten = graph.snapshot(kTenMm);
  if (ten.tracked_size != kExpectedComponent || ten.tracked_seeds != kExpectedSeedInComponent) {
    std::cerr << "10 mm 추적 component가 R1E와 다릅니다. size=" << ten.tracked_size
              << " seeds=" << ten.tracked_seeds << "\n";
    return 4;
  }
  const Critical critical = findCritical(graph);
  double induced = -1.0;
  {
    std::vector<Vec3> seed_points;
    seed_points.reserve(seed_indices.size());
    for (const int index : seed_indices) {
      seed_points.push_back(selected[static_cast<std::size_t>(index)]);
    }
    std::vector<char> all(seed_points.size(), 1);
    int tracked_local = 0;
    double local_best = distanceBetween(seed_points.front(), origin);
    for (int index = 1; index < static_cast<int>(seed_points.size()); ++index) {
      const double value = distanceBetween(seed_points[static_cast<std::size_t>(index)], origin);
      if (value < local_best) {
        local_best = value;
        tracked_local = index;
      }
    }
    RadiusGraph induced_graph(seed_points, all, tracked_local);
    const int min_key = searchKey(kSearchMin);
    const int max_key = searchKey(kSearchMax);
    if (!induced_graph.allSeeds(induced_graph.snapshot(toleranceAt(min_key))) &&
        induced_graph.allSeeds(induced_graph.snapshot(toleranceAt(max_key)))) {
      int low = min_key;
      int high = max_key;
      while (high - low > 1) {
        const int mid = (low + high) / 2;
        if (induced_graph.allSeeds(induced_graph.snapshot(toleranceAt(mid)))) {
          high = mid;
        } else {
          low = mid;
        }
      }
      induced = toleranceAt(high);
    }
  }
  Edge edge;
  std::vector<int> path;
  Dsu below(1);
  bool have_below = false;
  if (critical.resolved) {
    const double t_env = toleranceAt(critical.environment_key);
    const double t_below = toleranceAt(critical.environment_key - 1);
    edge = graph.bridge(t_below, t_env);
    if (edge.shoe >= 0) {
      path = graph.pathTo(toleranceAt(critical.environment_key), tracked, edge.environment);
    }
    below = graph.connect(t_below);
    have_below = true;
  }
  double plane_a = 0, plane_b = 0, plane_c = 0, plane_d = 0;
  const bool have_plane = readCoefficient(repro_dir + "/plane.json", "viewpoint_a", plane_a) &&
      readCoefficient(repro_dir + "/plane.json", "viewpoint_b", plane_b) &&
      readCoefficient(repro_dir + "/plane.json", "viewpoint_c", plane_c) &&
      readCoefficient(repro_dir + "/plane.json", "viewpoint_d", plane_d);
  auto heightOf = [&](const Vec3& point) {
    return plane_a * point.x + plane_b * point.y + plane_c * point.z + plane_d;
  };
  std::string normal_input = "UNRESOLVED";
  std::string normal_reason = "NOT_EVALUATED_FEASIBLE";
  double density_median = -1.0;
  double density_fraction = -1.0;
  double bridge_angle = -1.0;
  double interior_median = -1.0;
  double interior_p95 = -1.0;
  if (critical.separation == "MARGINAL" || critical.separation == "NOT_FEASIBLE") {
    normal_reason = "MEASURED";
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normals_estimator;
    auto cloud = toCloud(selected);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    normals_estimator.setInputCloud(cloud);
    normals_estimator.setSearchMethod(tree);
    normals_estimator.setKSearch(20);
    normals_estimator.setViewPoint(0.f, 0.f, 0.f);
    pcl::PointCloud<pcl::Normal> normals;
    normals_estimator.compute(normals);
    std::vector<double> tenth;
    int within = 0;
    std::vector<int> neighbors;
    std::vector<float> squared;
    for (const int index : seed_indices) {
      neighbors.clear();
      squared.clear();
      tree->nearestKSearch(index, 11, neighbors, squared);
      if (squared.size() < 11) {
        continue;
      }
      const double tenth_distance = std::sqrt(static_cast<double>(squared.back()));
      tenth.push_back(tenth_distance);
      within += tenth_distance <= kTenMm ? 1 : 0;
    }
    density_median = percentile(tenth, 0.5);
    density_fraction = tenth.empty() ? 0.0 : static_cast<double>(within) / static_cast<double>(tenth.size());
    const bool density_ok = density_median <= kTenMm && density_fraction >= 0.8;
    auto angleBetween = [&](int left, int right) {
      const auto& a = normals[static_cast<std::size_t>(left)];
      const auto& b = normals[static_cast<std::size_t>(right)];
      const double dot = static_cast<double>(a.normal_x) * b.normal_x + static_cast<double>(a.normal_y) * b.normal_y +
          static_cast<double>(a.normal_z) * b.normal_z;
      if (!std::isfinite(dot)) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      return std::acos(std::max(-1.0, std::min(1.0, dot)));
    };
    pcl::PointCloud<pcl::PointXYZ>::Ptr seed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const int index : seed_indices) {
      const Vec3& point = selected[static_cast<std::size_t>(index)];
      seed_cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
    }
    seed_cloud->width = static_cast<std::uint32_t>(seed_cloud->size());
    seed_cloud->height = 1;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr seed_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    seed_tree->setInputCloud(seed_cloud);
    std::vector<double> interior;
    for (int local = 0; local < static_cast<int>(seed_indices.size()); ++local) {
      neighbors.clear();
      squared.clear();
      seed_tree->nearestKSearch(local, 2, neighbors, squared);
      if (neighbors.size() < 2) {
        continue;
      }
      const double angle = angleBetween(seed_indices[static_cast<std::size_t>(local)], seed_indices[static_cast<std::size_t>(neighbors[1])]);
      if (std::isfinite(angle)) {
        interior.push_back(angle);
      }
    }
    interior_median = percentile(interior, 0.5);
    interior_p95 = percentile(interior, 0.95);
    if (edge.shoe >= 0 && edge.environment >= 0) {
      bridge_angle = angleBetween(edge.shoe, edge.environment);
    }
    if (!density_ok) {
      normal_input = "UNSUITABLE";
    } else if (!std::isfinite(bridge_angle) || interior.empty()) {
      normal_input = "UNRESOLVED";
      normal_reason = "ANGLE_MISSING";
    } else if (bridge_angle > interior_p95) {
      normal_input = "SUITABLE";
    } else if (bridge_angle <= interior_median) {
      normal_input = "UNSUITABLE";
    } else {
      normal_input = "UNRESOLVED";
      normal_reason = "ANGLE_BETWEEN_MEDIAN_AND_P95";
    }
  }
  std::filesystem::create_directories(out_dir);
  const std::string verdict = critical.separation == "UNRESOLVED" ? "INCONCLUSIVE" : "PASS_DIAGNOSIS";
  {
    std::ofstream output(out_dir + "/connectivity_metrics.json");
    output << "{\n";
    output << "  \"t_internal_definition\": \"full_prism_radius_graph\",\n";
    output << "  \"t_env_definition\": \"tracked_component_size_gt_2_times_seed_count\",\n";
    output << "  \"search_min_m\": " << jsonNumber(kSearchMin) << ",\n";
    output << "  \"search_max_m\": " << jsonNumber(kSearchMax) << ",\n";
    output << "  \"search_step_m\": " << jsonNumber(kSearchStep) << ",\n";
    output << "  \"margin_steps\": " << kMarginSteps << ",\n";
    output << "  \"seed_count\": " << seed_indices.size() << ",\n";
    output << "  \"tracked_index\": " << tracked << ",\n";
    output << "  \"ten_mm_component_points\": " << ten.tracked_size << ",\n";
    output << "  \"ten_mm_seed_points\": " << ten.tracked_seeds << ",\n";
    output << "  \"ten_mm_seed_component_count\": " << ten.seed_component_count << ",\n";
    output << "  \"ten_mm_cluster_count_min50\": " << ten.cluster_count << ",\n";
    output << "  \"t_internal_m\": " << (critical.resolved ? jsonNumber(toleranceAt(critical.internal_key)) : "null") << ",\n";
    output << "  \"t_env_m\": " << (critical.resolved ? jsonNumber(toleranceAt(critical.environment_key)) : "null") << ",\n";
    output << "  \"delta_m\": "
           << (critical.resolved ? jsonNumber(toleranceAt(critical.environment_key) - toleranceAt(critical.internal_key)) : "null")
           << ",\n";
    output << "  \"separation_interval\": \"" << separationInterval(critical) << "\",\n";
    output << "  \"distance_separation\": \"" << critical.separation << "\",\n";
    output << "  \"induced_seed_distance_m\": " << (induced > 0.0 ? jsonNumber(induced) : "null") << ",\n";
    output << "  \"bridge_edge_length_m\": " << (edge.shoe >= 0 ? jsonNumber(edge.length) : "null") << ",\n";
    output << "  \"bridge_shoe_height_m\": "
           << (edge.shoe >= 0 && have_plane ? jsonNumber(heightOf(selected[static_cast<std::size_t>(edge.shoe)])) : "null")
           << ",\n";
    output << "  \"bridge_environment_height_m\": "
           << (edge.environment >= 0 && have_plane ? jsonNumber(heightOf(selected[static_cast<std::size_t>(edge.environment)])) : "null")
           << "\n";
    output << "}\n";
  }
  {
    std::ofstream output(out_dir + "/tolerance_sweep.csv");
    output << "tolerance_m,component_count,cluster_count_min50,tracked_component_points,shoe_membership,"
              "environment_merge,shoe_fragment_count,bbox_min_x,bbox_min_y,bbox_min_z,bbox_max_x,bbox_max_y,bbox_max_z,"
              "dim_x,dim_y,dim_z\n";
    output.setf(std::ios::fmtflags(0), std::ios::floatfield);
    output.precision(17);
    for (const double tolerance : kSweep) {
      const Snapshot snapshot = graph.snapshot(tolerance);
      const double membership = static_cast<double>(snapshot.tracked_seeds) / static_cast<double>(seed_indices.size());
      output << tolerance << "," << snapshot.component_count << "," << snapshot.cluster_count << ","
             << snapshot.tracked_size << "," << membership << "," << (snapshot.environment_merged ? 1 : 0) << ","
             << snapshot.seed_component_count << "," << snapshot.bbox_min.x << "," << snapshot.bbox_min.y << ","
             << snapshot.bbox_min.z << "," << snapshot.bbox_max.x << "," << snapshot.bbox_max.y << ","
             << snapshot.bbox_max.z << "," << (snapshot.bbox_max.x - snapshot.bbox_min.x) << ","
             << (snapshot.bbox_max.y - snapshot.bbox_min.y) << "," << (snapshot.bbox_max.z - snapshot.bbox_min.z) << "\n";
    }
  }
  {
    std::ofstream output(out_dir + "/bridge_points.csv");
    output << "role,x,y,z,height_m\n";
    output.setf(std::ios::fmtflags(0), std::ios::floatfield);
    output.precision(17);
    auto writePoint = [&](const std::string& role, int index) {
      if (index < 0) {
        return;
      }
      const Vec3& point = selected[static_cast<std::size_t>(index)];
      output << role << "," << point.x << "," << point.y << "," << point.z << ","
             << (have_plane ? heightOf(point) : 0.0) << "\n";
    };
    writePoint("shoe_endpoint", edge.shoe);
    writePoint("environment_endpoint", edge.environment);
    for (const int index : path) {
      writePoint("path", index);
    }
  }
  {
    std::ofstream output(out_dir + "/normal_suitability.json");
    output << "{\n";
    output << "  \"normal_constrained_clustering_input\": \"" << normal_input << "\",\n";
    output << "  \"reason\": \"" << normal_reason << "\",\n";
    output << "  \"normal_estimation_k\": 20,\n";
    output << "  \"density_10nn_median_m\": " << (density_median >= 0.0 ? jsonNumber(density_median) : "null") << ",\n";
    output << "  \"density_fraction_within_0_01\": " << (density_fraction >= 0.0 ? jsonNumber(density_fraction) : "null") << ",\n";
    output << "  \"bridge_normal_angle_rad\": " << (std::isfinite(bridge_angle) && bridge_angle >= 0.0 ? jsonNumber(bridge_angle) : "null")
           << ",\n";
    output << "  \"interior_normal_angle_median_rad\": " << (interior_median >= 0.0 ? jsonNumber(interior_median) : "null") << ",\n";
    output << "  \"interior_normal_angle_p95_rad\": " << (interior_p95 >= 0.0 ? jsonNumber(interior_p95) : "null") << ",\n";
    output << "  \"segmentation_ran\": false\n";
    output << "}\n";
  }
  {
    std::ofstream output(out_dir + "/verdict.json");
    output << "{\n";
    output << "  \"gate\": \"P-008-R1F\",\n";
    output << "  \"verdict\": \"" << verdict << "\",\n";
    output << "  \"distance_separation\": \"" << critical.separation << "\",\n";
    output << "  \"separation_interval\": \"" << separationInterval(critical) << "\",\n";
    output << "  \"normal_constrained_clustering_input\": \"" << normal_input << "\",\n";
    output << "  \"parameters_changed\": false,\n";
    output << "  \"parameters_frozen\": false,\n";
    output << "  \"capture_02_05_used\": false\n";
    output << "}\n";
  }
  writeBin(out_dir + "/stage_selected.bin", selected);
  writeBin(out_dir + "/stage_seed.bin", [&] {
    std::vector<Vec3> points;
    for (const int index : seed_indices) {
      points.push_back(selected[static_cast<std::size_t>(index)]);
    }
    return points;
  }());
  writeBin(out_dir + "/stage_tracked_10mm.bin", graph.componentPoints(graph.connect(kTenMm), tracked));
  if (have_below && edge.shoe >= 0) {
    writeBin(out_dir + "/stage_bridge_shoe.bin", graph.componentPoints(below, edge.shoe));
    writeBin(out_dir + "/stage_bridge_env.bin", graph.componentPoints(below, edge.environment));
    std::vector<Vec3> path_points;
    for (const int index : path) {
      path_points.push_back(selected[static_cast<std::size_t>(index)]);
    }
    writeBin(out_dir + "/stage_path.bin", path_points);
  }
  std::cout << "verdict=" << verdict << " separation=" << critical.separation
            << " t_internal=" << (critical.resolved ? toleranceAt(critical.internal_key) : -1.0)
            << " t_env=" << (critical.resolved ? toleranceAt(critical.environment_key) : -1.0)
            << " normal=" << normal_input << "\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--self-test") {
    return selfTest();
  }
  if (argc != 3) {
    std::cerr << "사용: p008r1f_connectivity <repro_dir> <output_dir> | --self-test\n";
    return 2;
  }
  return runAnalysis(argv[1], argv[2]);
}
