// 저장된 관절 후보만으로 coupon 충돌 제약을 넣어 다시 탐색한다. IK와 중간점은 추가하지 않는다.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sys/resource.h>

#include <Eigen/Geometry>

#include <openssl/sha.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometric_shapes/shapes.h>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <descartes_light/core/waypoint_sampler.h>
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>
#include <descartes_light/solvers/ladder_graph/ladder_graph_solver.h>

#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

namespace
{
const int kRunCount = 21;
const int kCandidateCount = 256;
const int kSampleDenominator = 100;
const int kTargetRuns[] = {1, 2, 3, 4, 5, 6, 11, 12, 15, 16, 17, 18, 19};
const double kModelPositionM = 1.0e-6;
const double kModelOrientationRad = 1.0e-6;
const double kPathPositionM = 0.001;
const double kPathOrientationRad = 0.01;
const double kLimitBoundaryRad = 1.0e-12;
const double kInternalWallS = 1790.0;
const double kBoundaryOverallS = 550.0;
const double kPrepS = 10.0;
const double kFilterS = 20.0;
const double kLadderS = 150.0;
const double kLinearS = 10.0;
const double kPostS = 20.0;
const double kSaveS = 20.0;
const double kFullOverallS = 757.0;
const double kFullPrepS = 10.0;
const double kFullFilterS = 35.0;
const double kFullLadderS = 680.0;
const double kFullPostS = 22.0;
const double kFullSaveS = 10.0;
const double kRestoreOverallS = 180.0;
const double kRestoreFilterS = 90.0;
const double kRestoreInspectS = 50.0;
const char * kInsertsSha = "ff1f59d4275cdee2e41f5b8ef0da3182480204eb32d2372e1eb65ef4dd40f153";
const char * kSelectedSha = "eac7465345b5601422bcaa0225a607e885be177f6c5da30d75cd6925e8b066f4";
std::atomic<std::uint64_t> g_accepted_edges{0};
const char * kTransitionsSha = "cc21e6083a9ecb0c740382ccf3f9b8a7e3e9bdaea64017389525cbc04cd821c4";
std::chrono::steady_clock::time_point g_process_start;
double g_overall_s = 1.0e9;
double g_phase_wall_s = 1790.0;
const rlim_t kAddressSpaceBytes = 8ULL * 1024ULL * 1024ULL * 1024ULL;
const char * kCouponId = "complex_surface_coupon";
const char * kRunsSha = "4d51b28f0851114b8d36036d0fb96bb1681bf03f89ac31a47bf7d3a18eecfb26";
const char * kUrdfSha = "a21bf5fb70b3a1745bf2e4816e0f43654539737ba7f2585939ec773d159778d8";
const char * kSrdfSha = "ee0d940b486bca52534d5b112c1020e0eaa542130c1301af8dc2fda28c757dbb";
const char * kMeshSha = "ea78058ca33866ef3c69764be48f90da4a6f2491455c9077c514ccecd6acbc16";
const char * kAcceptanceSha = "97fa8c989cfae92fc998003ceb010e4c585642fe13b157330a04cb162e610e44";
const double kDatumM[3] = {0.3244333936553, -0.0499999998932, 0.11133404082435};
const char * kJointNames[6] = {
  "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

struct ExpectedPair
{
  const char * first;
  const char * second;
};

const ExpectedPair kExpectedPairs[] = {
  {"base_link", "base_link_inertia"},
  {"base_link_inertia", "shoulder_link"},
  {"forearm_link", "upper_arm_link"},
  {"forearm_link", "wrist_1_link"},
  {"shoulder_link", "upper_arm_link"},
  {"tool0", "wrist_3_link"},
  {"wrist_1_link", "wrist_2_link"},
  {"wrist_2_link", "wrist_3_link"},
  {"tool0", "wrist_1_link"},
  {"tool0", "wrist_2_link"},
  {"wrist_1_link", "wrist_3_link"}};

struct ExpectedMesh
{
  const char * link;
  const char * uri;
  const char * sha256;
};

const ExpectedMesh kExpectedMeshes[] = {
  {"base_link_inertia", "package://ur_description/meshes/ur5e/collision/base.stl",
    "9a145e0d46f2130afdf2a2e8825a00a929870c4c3d6d8e4d1adc5f04db3aac1b"},
  {"shoulder_link", "package://ur_description/meshes/ur5e/collision/shoulder.stl",
    "ceb92532177daa77682f5fbd628e01c2137d168f949a7a706ce1dabe9f002387"},
  {"upper_arm_link", "package://ur_description/meshes/ur5e/collision/upperarm.stl",
    "ee893044caf00075cb55b4cf666d1f1311c7979786212a501009f33bee945209"},
  {"forearm_link", "package://ur_description/meshes/ur5e/collision/forearm.stl",
    "2e7423cab807c34160ec4f770daee5e747d70e777eb01b7beeace2b8c5751816"},
  {"wrist_1_link", "package://ur_description/meshes/ur5e/collision/wrist1.stl",
    "f8c9f9337b6fd98c75f052e96de10e14a107ddb6874ba6b904e546f8a4e4f43a"},
  {"wrist_2_link", "package://ur_description/meshes/ur5e/collision/wrist2.stl",
    "2964a63f60ce3e3cf3ad55bcf190d7876d50e373cb64b70a57cea5885eaf3c86"},
  {"wrist_3_link", "package://ur_description/meshes/ur5e/collision/wrist3.stl",
    "83b3666b4ae2badd54af0d2c25a921682ecbc29e849eec646c3ed55fb74c78a3"}};

struct Candidate
{
  int source_code = 0;
  int kinematic_class_id = 0;
  Eigen::VectorXd joints;
};

struct Waypoint
{
  int sequence_id = -1;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  std::vector<Candidate> candidates;
};

struct Insert
{
  int from_sequence_id = -1;
  int to_sequence_id = -1;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  std::vector<Candidate> valid;
};

struct Rung
{
  bool inserted = false;
  int sequence_id = -1;
  int from_sequence_id = -1;
  int to_sequence_id = -1;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  std::vector<Candidate> candidates;
  std::vector<std::size_t> survivors;
};

struct Counts
{
  std::atomic<std::uint64_t> accuracy_pass{0};
  std::atomic<std::uint64_t> accuracy_fail{0};
  std::atomic<std::uint64_t> accuracy_not_evaluated{0};
  std::atomic<std::uint64_t> self_clear{0};
  std::atomic<std::uint64_t> self_collision{0};
  std::atomic<std::uint64_t> self_not_evaluated{0};
  std::atomic<std::uint64_t> self_error{0};
  std::atomic<std::uint64_t> coupon_clear{0};
  std::atomic<std::uint64_t> coupon_collision{0};
  std::atomic<std::uint64_t> coupon_not_evaluated{0};
  std::atomic<std::uint64_t> coupon_error{0};
  std::atomic<std::uint64_t> nonfinite{0};
  std::atomic<std::uint64_t> limit_fail{0};
  std::atomic<std::uint64_t> evaluations{0};
};

struct SceneInfo
{
  int vertices = 0;
  int triangles = 0;
  int always_pairs = 0;
  int explicit_entries = 0;
  int padding_mismatch = 0;
  int scale_mismatch = 0;
  double datum[3] = {0.0, 0.0, 0.0};
  bool pose_match = false;
  bool mesh_match = false;
  bool ok = false;
};

struct Shared
{
  moveit::core::RobotModelConstPtr model;
  const moveit::core::JointModelGroup * group = nullptr;
  collision_detection::AllowedCollisionMatrix acm;
  collision_detection::CollisionRequest request;
  std::vector<double> vertices_m;
  std::vector<std::uint32_t> triangles;
  Eigen::MatrixX2d limits;
  std::chrono::steady_clock::time_point started;
  std::atomic<bool> time_exceeded{false};
  std::atomic<bool> setup_failed{false};
  std::mutex scene_mutex;
  std::vector<SceneInfo> scenes;
  SceneInfo reference;
  Counts rung;
  Counts edge;
};

Shared * g_shared = nullptr;
std::string g_out_path;

class RecordedJointSampler : public descartes_light::WaypointSampler<double>
{
public:
  explicit RecordedJointSampler(std::vector<descartes_light::State<double>::ConstPtr> states)
  : states_(std::move(states))
  {
  }

  std::vector<descartes_light::StateSample<double>> sample() const override
  {
    std::vector<descartes_light::StateSample<double>> samples;
    samples.reserve(states_.size());
    for (const auto & state : states_)
    {
      samples.emplace_back(state, 0.0);
    }
    return samples;
  }

private:
  std::vector<descartes_light::State<double>::ConstPtr> states_;
};

struct LocalScene
{
  planning_scene::PlanningScenePtr scene;
  moveit::core::RobotState state;
  bool ready = false;

  explicit LocalScene(const moveit::core::RobotModelConstPtr & model)
  : state(model)
  {
  }
};

std::string read_file(const std::string & path)
{
  std::ifstream input(path);
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

std::string sha256_file(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input)
  {
    return "";
  }
  SHA256_CTX context;
  SHA256_Init(&context);
  char buffer[65536];
  while (input)
  {
    input.read(buffer, sizeof(buffer));
    const std::streamsize count = input.gcount();
    if (count > 0)
    {
      SHA256_Update(&context, buffer, static_cast<std::size_t>(count));
    }
  }
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256_Final(digest, &context);
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned char byte : digest)
  {
    output << std::setw(2) << static_cast<int>(byte);
  }
  return output.str();
}

void write_number(std::ostream & output, double value)
{
  if (!std::isfinite(value))
  {
    output << "null";
    return;
  }
  output << std::setprecision(17) << value;
}

template <typename T>
bool read_pod(std::istream & input, T * value)
{
  input.read(reinterpret_cast<char *>(value), sizeof(T));
  return static_cast<bool>(input);
}

bool address_limit_ok()
{
  struct rlimit limit;
  if (getrlimit(RLIMIT_AS, &limit) != 0)
  {
    return false;
  }
  return limit.rlim_cur == kAddressSpaceBytes && limit.rlim_max == kAddressSpaceBytes;
}

double process_elapsed()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - g_process_start).count();
}

bool timed_out()
{
  const double phase_elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - g_shared->started).count();
  if (phase_elapsed > g_phase_wall_s || process_elapsed() > g_overall_s)
  {
    g_shared->time_exceeded.store(true);
    return true;
  }
  return g_shared->time_exceeded.load();
}

bool begin_phase(double cap)
{
  if (!(g_overall_s - process_elapsed() >= cap))
  {
    return false;
  }
  g_shared->started = std::chrono::steady_clock::now();
  g_shared->time_exceeded.store(false);
  g_phase_wall_s = cap;
  return true;
}

const std::vector<int> & sample_order()
{
  static const std::vector<int> order = []() {
    std::vector<int> values;
    values.push_back(50);
    for (int sample = 0; sample <= kSampleDenominator; ++sample)
    {
      if (sample != 50)
      {
        values.push_back(sample);
      }
    }
    return values;
  }();
  return order;
}

double rotation_error_rad(const Eigen::Matrix3d & target, const Eigen::Matrix3d & actual)
{
  const Eigen::Matrix3d relative = target.transpose() * actual;
  double cosine = (relative.trace() - 1.0) * 0.5;
  cosine = std::max(-1.0, std::min(1.0, cosine));
  return std::acos(cosine);
}

Eigen::Isometry3d tool_in_base(const moveit::core::RobotState & state)
{
  return state.getGlobalLinkTransform("base_link").inverse() * state.getGlobalLinkTransform("tool0");
}

bool within_limits(const Eigen::VectorXd & joints)
{
  for (int index = 0; index < joints.size(); ++index)
  {
    if (!std::isfinite(joints[index]) ||
        joints[index] < g_shared->limits(index, 0) - kLimitBoundaryRad ||
        joints[index] > g_shared->limits(index, 1) + kLimitBoundaryRad)
    {
      return false;
    }
  }
  return true;
}

bool same_vector(const Eigen::VectorXd & left, const Eigen::VectorXd & right)
{
  if (left.size() != right.size())
  {
    return false;
  }
  for (int index = 0; index < left.size(); ++index)
  {
    if (left[index] != right[index])
    {
      return false;
    }
  }
  return true;
}

std::string resolve_package_uri(const std::string & uri)
{
  const std::string prefix = "package://";
  if (uri.compare(0, prefix.size(), prefix) != 0)
  {
    return "";
  }
  const std::size_t slash = uri.find('/', prefix.size());
  if (slash == std::string::npos)
  {
    return "";
  }
  try
  {
    const std::string package = uri.substr(prefix.size(), slash - prefix.size());
    return ament_index_cpp::get_package_share_directory(package) + "/" + uri.substr(slash + 1);
  }
  catch (const std::exception &)
  {
    return "";
  }
}

bool robot_mesh_hashes_ok(const urdf::ModelInterface & urdf, std::string * error)
{
  for (const ExpectedMesh & expected : kExpectedMeshes)
  {
    const auto link = urdf.getLink(expected.link);
    if (!link || link->collision_array.size() != 1 || !link->collision_array[0]->geometry)
    {
      *error = "robot_mesh_missing";
      return false;
    }
    const auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->collision_array[0]->geometry);
    if (!mesh || mesh->filename != expected.uri)
    {
      *error = "robot_mesh_uri";
      return false;
    }
    const std::string digest = sha256_file(resolve_package_uri(mesh->filename));
    if (digest != expected.sha256)
    {
      *error = std::string("robot_mesh_hash ") + expected.link;
      return false;
    }
  }
  return true;
}

bool load_runs(const std::string & path, std::vector<std::vector<Waypoint>> * runs)
{
  std::ifstream input(path, std::ios::binary);
  char magic[5] = {};
  input.read(magic, 5);
  if (!input || std::string(magic, 5) != "URPC1")
  {
    return false;
  }
  std::uint32_t run_count = 0;
  if (!read_pod(input, &run_count) || run_count != static_cast<std::uint32_t>(kRunCount))
  {
    return false;
  }
  runs->resize(kRunCount);
  int process_count = 0;
  for (int run_id = 0; run_id < kRunCount; ++run_id)
  {
    std::int32_t stored_run = 0;
    std::uint32_t waypoint_count = 0;
    if (!read_pod(input, &stored_run) || !read_pod(input, &waypoint_count) || stored_run != run_id)
    {
      return false;
    }
    auto & run = (*runs)[static_cast<std::size_t>(run_id)];
    run.resize(waypoint_count);
    for (std::uint32_t waypoint_index = 0; waypoint_index < waypoint_count; ++waypoint_index)
    {
      Waypoint & waypoint = run[waypoint_index];
      std::int32_t sequence_id = 0;
      if (!read_pod(input, &sequence_id))
      {
        return false;
      }
      waypoint.sequence_id = sequence_id;
      double position[3];
      double orientation[4];
      input.read(reinterpret_cast<char *>(position), sizeof(position));
      input.read(reinterpret_cast<char *>(orientation), sizeof(orientation));
      waypoint.position = Eigen::Vector3d(position[0], position[1], position[2]);
      waypoint.orientation = Eigen::Quaterniond(orientation[3], orientation[0], orientation[1], orientation[2]);
      std::uint32_t candidate_count = 0;
      if (!read_pod(input, &candidate_count) || candidate_count != static_cast<std::uint32_t>(kCandidateCount))
      {
        return false;
      }
      waypoint.candidates.resize(kCandidateCount);
      for (int candidate_index = 0; candidate_index < kCandidateCount; ++candidate_index)
      {
        Candidate & candidate = waypoint.candidates[static_cast<std::size_t>(candidate_index)];
        std::int32_t class_id = 0;
        std::uint8_t source = 0;
        if (!read_pod(input, &class_id) || !read_pod(input, &source) || (source != 0 && source != 1))
        {
          return false;
        }
        candidate.kinematic_class_id = class_id;
        candidate.source_code = source;
        double joints[6];
        input.read(reinterpret_cast<char *>(joints), sizeof(joints));
        candidate.joints = Eigen::VectorXd(6);
        for (int joint_index = 0; joint_index < 6; ++joint_index)
        {
          if (!std::isfinite(joints[joint_index]))
          {
            return false;
          }
          candidate.joints[joint_index] = joints[joint_index];
        }
      }
      if (!waypoint.position.allFinite() || !(waypoint.orientation.norm() > 0.0))
      {
        return false;
      }
      waypoint.orientation.normalize();
      ++process_count;
    }
  }
  return process_count == 721 && input && input.peek() == EOF;
}

bool load_inserts(const std::string & path, std::map<int, std::vector<Insert>> * inserts)
{
  std::ifstream input(path, std::ios::binary);
  char magic[4] = {};
  input.read(magic, 4);
  std::uint32_t run_count = 0;
  if (!input || std::string(magic, 4) != "R3IN" || !read_pod(input, &run_count) || run_count != 2)
  {
    return false;
  }
  for (std::uint32_t index = 0; index < run_count; ++index)
  {
    std::int32_t run_id = 0;
    std::int32_t insert_count = 0;
    if (!read_pod(input, &run_id) || !read_pod(input, &insert_count) || insert_count < 1)
    {
      return false;
    }
    if (run_id != 6 && run_id != 11)
    {
      return false;
    }
    auto & run_inserts = (*inserts)[run_id];
    run_inserts.resize(static_cast<std::size_t>(insert_count));
    for (Insert & insert : run_inserts)
    {
      std::int32_t from_id = 0;
      std::int32_t to_id = 0;
      std::uint32_t valid_count = 0;
      double position[3];
      double orientation[4];
      if (!read_pod(input, &from_id) || !read_pod(input, &to_id))
      {
        return false;
      }
      input.read(reinterpret_cast<char *>(position), sizeof(position));
      input.read(reinterpret_cast<char *>(orientation), sizeof(orientation));
      if (!read_pod(input, &valid_count) || valid_count != 256)
      {
        return false;
      }
      insert.from_sequence_id = from_id;
      insert.to_sequence_id = to_id;
      insert.position = Eigen::Vector3d(position[0], position[1], position[2]);
      insert.orientation = Eigen::Quaterniond(orientation[3], orientation[0], orientation[1], orientation[2]);
      if (!insert.position.allFinite() || !(insert.orientation.norm() > 0.0))
      {
        return false;
      }
      insert.valid.resize(valid_count);
      for (Candidate & candidate : insert.valid)
      {
        std::int32_t class_id = 0;
        std::uint8_t source = 0;
        if (!read_pod(input, &class_id) || !read_pod(input, &source) || (source != 0 && source != 1))
        {
          return false;
        }
        candidate.kinematic_class_id = class_id;
        candidate.source_code = source;
        double joints[6];
        input.read(reinterpret_cast<char *>(joints), sizeof(joints));
        candidate.joints = Eigen::VectorXd(6);
        for (int joint_index = 0; joint_index < 6; ++joint_index)
        {
          if (!std::isfinite(joints[joint_index]))
          {
            return false;
          }
          candidate.joints[joint_index] = joints[joint_index];
        }
      }
    }
  }
  return input && input.peek() == EOF && inserts->count(6) == 1 && inserts->count(11) == 1;
}

bool load_coupon(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  char magic[8] = {};
  input.read(magic, 8);
  std::uint32_t accepted = 0;
  std::uint32_t vertex_count = 0;
  std::uint32_t triangle_count = 0;
  if (!input || std::string(magic, 5) != "COUP1" || !read_pod(input, &accepted) ||
      !read_pod(input, &vertex_count) || !read_pod(input, &triangle_count) || accepted != 1 ||
      vertex_count != 413 || triangle_count != 564)
  {
    return false;
  }
  g_shared->vertices_m.resize(static_cast<std::size_t>(vertex_count) * 3);
  g_shared->triangles.resize(static_cast<std::size_t>(triangle_count) * 3);
  input.read(reinterpret_cast<char *>(g_shared->vertices_m.data()), g_shared->vertices_m.size() * sizeof(double));
  input.read(reinterpret_cast<char *>(g_shared->triangles.data()), g_shared->triangles.size() * sizeof(std::uint32_t));
  return static_cast<bool>(input) && input.peek() == EOF;
}

using NamePair = std::pair<std::string, std::string>;

NamePair ordered_pair(const std::string & left, const std::string & right)
{
  return left < right ? NamePair{left, right} : NamePair{right, left};
}

SceneInfo inspect_scene(const planning_scene::PlanningScene & scene)
{
  SceneInfo info;
  const auto object = scene.getWorld()->getObject(kCouponId);
  const shapes::Mesh * loaded = object && object->shapes_.size() == 1 ?
    dynamic_cast<const shapes::Mesh *>(object->shapes_[0].get()) : nullptr;
  info.vertices = loaded ? static_cast<int>(loaded->vertex_count) : 0;
  info.triangles = loaded ? static_cast<int>(loaded->triangle_count) : 0;
  info.mesh_match = loaded != nullptr && loaded->vertex_count == 413 && loaded->triangle_count == 564;
  if (info.mesh_match)
  {
    for (std::size_t index = 0; index < g_shared->vertices_m.size(); ++index)
    {
      if (loaded->vertices[index] != g_shared->vertices_m[index])
      {
        info.mesh_match = false;
      }
    }
    for (std::size_t index = 0; index < g_shared->triangles.size(); ++index)
    {
      if (loaded->triangles[index] != g_shared->triangles[index])
      {
        info.mesh_match = false;
      }
    }
  }
  if (object && object->pose_.translation().size() == 3)
  {
    info.datum[0] = object->pose_.translation().x();
    info.datum[1] = object->pose_.translation().y();
    info.datum[2] = object->pose_.translation().z();
    info.pose_match = info.datum[0] == kDatumM[0] && info.datum[1] == kDatumM[1] &&
      info.datum[2] == kDatumM[2] &&
      object->pose_.rotation() == Eigen::Matrix3d::Identity() &&
      object->shape_poses_.size() == 1 &&
      object->shape_poses_[0].matrix() == Eigen::Isometry3d::Identity().matrix();
  }
  const auto & acm = scene.getAllowedCollisionMatrix();
  std::map<NamePair, bool> expected;
  for (const ExpectedPair & pair : kExpectedPairs)
  {
    expected[ordered_pair(pair.first, pair.second)] = true;
  }
  collision_detection::AllowedCollision::Type type;
  for (const auto & pair : expected)
  {
    if (acm.getEntry(pair.first.first, pair.first.second, type) &&
        type == collision_detection::AllowedCollision::ALWAYS)
    {
      ++info.always_pairs;
    }
  }
  const auto names = scene.getRobotModel()->getLinkModelNames();
  for (std::size_t left = 0; left < names.size(); ++left)
  {
    for (std::size_t right = left + 1; right < names.size(); ++right)
    {
      if (acm.getEntry(names[left], names[right], type))
      {
        ++info.explicit_entries;
      }
    }
  }
  const auto * environment = scene.getCollisionEnv().get();
  for (const std::string & name : names)
  {
    if (environment->getLinkPadding(name) != 0.0)
    {
      ++info.padding_mismatch;
    }
    if (environment->getLinkScale(name) != 1.0)
    {
      ++info.scale_mismatch;
    }
  }
  info.ok = scene.getWorld()->size() == 1 && info.mesh_match && info.pose_match &&
    info.always_pairs == 11 && info.explicit_entries == 11 &&
    info.padding_mismatch == 0 && info.scale_mismatch == 0;
  return info;
}

bool scenes_match(const SceneInfo & left, const SceneInfo & right)
{
  return left.ok && right.ok && left.vertices == right.vertices && left.triangles == right.triangles &&
    left.always_pairs == right.always_pairs && left.explicit_entries == right.explicit_entries &&
    left.datum[0] == right.datum[0] && left.datum[1] == right.datum[1] && left.datum[2] == right.datum[2];
}

LocalScene & local_scene()
{
  thread_local std::unique_ptr<LocalScene> local;
  if (!local)
  {
    local = std::make_unique<LocalScene>(g_shared->model);
    auto scene = std::make_shared<planning_scene::PlanningScene>(g_shared->model);
    auto raw = std::make_shared<shapes::Mesh>(413, 564);
    std::copy(g_shared->vertices_m.begin(), g_shared->vertices_m.end(), raw->vertices);
    for (std::size_t index = 0; index < g_shared->triangles.size(); ++index)
    {
      raw->triangles[index] = g_shared->triangles[index];
    }
    Eigen::Isometry3d object_pose = Eigen::Isometry3d::Identity();
    object_pose.translation() << kDatumM[0], kDatumM[1], kDatumM[2];
    scene->getWorldNonConst()->addToObject(kCouponId, object_pose, raw, Eigen::Isometry3d::Identity());
    const SceneInfo info = inspect_scene(*scene);
    {
      std::lock_guard<std::mutex> guard(g_shared->scene_mutex);
      if (g_shared->scenes.empty())
      {
        g_shared->reference = info;
      }
      g_shared->scenes.push_back(info);
      if (!scenes_match(info, g_shared->reference))
      {
        g_shared->setup_failed.store(true);
      }
    }
    local->scene = scene;
    local->ready = info.ok;
    if (!info.ok)
    {
      g_shared->setup_failed.store(true);
    }
  }
  return *local;
}

enum class Probe
{
  CLEAR,
  HIT,
  ERROR
};

Probe probe(bool self_collision, moveit::core::RobotState & state)
{
  collision_detection::CollisionResult result;
  LocalScene & local = local_scene();
  if (!local.ready || g_shared->setup_failed.load())
  {
    return Probe::ERROR;
  }
  try
  {
    if (self_collision)
    {
      local.scene->getCollisionEnv()->checkSelfCollision(
        g_shared->request, result, state, g_shared->acm);
    }
    else
    {
      local.scene->getCollisionEnv()->checkRobotCollision(
        g_shared->request, result, state, g_shared->acm);
    }
  }
  catch (const std::exception &)
  {
    return Probe::ERROR;
  }
  return result.collision ? Probe::HIT : Probe::CLEAR;
}

bool pose_error(
  const Eigen::VectorXd & joints,
  const Eigen::Vector3d & target_position,
  const Eigen::Quaterniond & target_orientation,
  double * position_error,
  double * orientation_error)
{
  if (!joints.allFinite() || !target_position.allFinite())
  {
    return false;
  }
  LocalScene & local = local_scene();
  std::vector<double> query(joints.data(), joints.data() + joints.size());
  local.state.setJointGroupPositions(g_shared->group, query);
  local.state.update();
  const Eigen::Isometry3d actual = tool_in_base(local.state);
  const Eigen::Matrix3d target_matrix = target_orientation.toRotationMatrix();
  if (!target_matrix.allFinite() || !actual.translation().allFinite() || !actual.rotation().allFinite())
  {
    return false;
  }
  *position_error = (actual.translation() - target_position).norm();
  *orientation_error = rotation_error_rad(target_matrix, actual.rotation());
  return std::isfinite(*position_error) && std::isfinite(*orientation_error);
}

void add_not_evaluated(Counts * counts, std::uint64_t count, bool self_only)
{
  counts->accuracy_not_evaluated.fetch_add(count);
  counts->self_not_evaluated.fetch_add(count);
  if (!self_only)
  {
    counts->coupon_not_evaluated.fetch_add(count);
  }
}

bool sample_accuracy(
  const Eigen::VectorXd & start_joints,
  const Eigen::VectorXd & end_joints,
  const Eigen::Vector3d & start_position,
  const Eigen::Vector3d & end_position,
  const Eigen::Quaterniond & start_orientation,
  const Eigen::Quaterniond & end_orientation,
  double t_start,
  double t_end,
  int sample_index,
  Eigen::VectorXd * joints_out)
{
  const double local = static_cast<double>(sample_index) / static_cast<double>(kSampleDenominator);
  const double parameter = (1.0 - local) * t_start + local * t_end;
  const Eigen::VectorXd joints = (1.0 - local) * start_joints + local * end_joints;
  *joints_out = joints;
  if (!std::isfinite(parameter) || !joints.allFinite())
  {
    return false;
  }
  const Eigen::Vector3d target_position = (1.0 - parameter) * start_position + parameter * end_position;
  const Eigen::Quaterniond target_orientation = start_orientation.slerp(parameter, end_orientation);
  double position_error = 0.0;
  double orientation_error = 0.0;
  if (!pose_error(joints, target_position, target_orientation, &position_error, &orientation_error))
  {
    return false;
  }
  return position_error <= kPathPositionM && orientation_error <= kPathOrientationRad;
}

void apply_collision(Counts * counts, const Eigen::VectorXd & joints, bool * self_hit, bool * coupon_hit, bool * error)
{
  LocalScene & local = local_scene();
  std::vector<double> query(joints.data(), joints.data() + joints.size());
  local.state.setJointGroupPositions(g_shared->group, query);
  local.state.update();
  const Probe self = probe(true, local.state);
  if (self == Probe::ERROR)
  {
    counts->self_error.fetch_add(1);
    counts->coupon_not_evaluated.fetch_add(1);
    *error = true;
    return;
  }
  if (self == Probe::HIT)
  {
    counts->self_collision.fetch_add(1);
    counts->coupon_not_evaluated.fetch_add(1);
    *self_hit = true;
    return;
  }
  counts->self_clear.fetch_add(1);
  const Probe coupon = probe(false, local.state);
  if (coupon == Probe::ERROR)
  {
    counts->coupon_error.fetch_add(1);
    *error = true;
    return;
  }
  if (coupon == Probe::HIT)
  {
    counts->coupon_collision.fetch_add(1);
    *coupon_hit = true;
    return;
  }
  counts->coupon_clear.fetch_add(1);
}

class CollisionEdgeEvaluator : public descartes_light::EdgeEvaluator<double>
{
public:
  CollisionEdgeEvaluator(
    Eigen::Vector3d start_position,
    Eigen::Vector3d end_position,
    Eigen::Quaterniond start_orientation,
    Eigen::Quaterniond end_orientation,
    double t_start,
    double t_end,
    descartes_light::EdgeEvaluator<double>::ConstPtr distance)
  : start_position_(std::move(start_position)),
    end_position_(std::move(end_position)),
    start_orientation_(std::move(start_orientation)),
    end_orientation_(std::move(end_orientation)),
    t_start_(t_start),
    t_end_(t_end),
    distance_(std::move(distance))
  {
  }

  std::pair<bool, double> evaluate(
    const descartes_light::State<double> & start,
    const descartes_light::State<double> & end) const override
  {
    g_shared->edge.evaluations.fetch_add(1);
    if (timed_out() || g_shared->setup_failed.load())
    {
      return std::make_pair(false, 0.0);
    }
    if (!start.values.allFinite() || !end.values.allFinite())
    {
      g_shared->edge.nonfinite.fetch_add(1);
      add_not_evaluated(&g_shared->edge, static_cast<std::uint64_t>(sample_order().size()), false);
      return std::make_pair(false, 0.0);
    }
    if (!within_limits(start.values) || !within_limits(end.values))
    {
      g_shared->edge.limit_fail.fetch_add(1);
      add_not_evaluated(&g_shared->edge, static_cast<std::uint64_t>(sample_order().size()), false);
      return std::make_pair(false, 0.0);
    }
    for (std::size_t order_index = 0; order_index < sample_order().size(); ++order_index)
    {
      if (timed_out())
      {
        const auto remaining = sample_order().size() - order_index;
        g_shared->edge.accuracy_not_evaluated.fetch_add(remaining);
        g_shared->edge.self_not_evaluated.fetch_add(sample_order().size());
        g_shared->edge.coupon_not_evaluated.fetch_add(sample_order().size());
        return std::make_pair(false, 0.0);
      }
      const int sample_index = sample_order()[order_index];
      Eigen::VectorXd joints;
      const bool accurate = sample_accuracy(
        start.values, end.values, start_position_, end_position_, start_orientation_, end_orientation_,
        t_start_, t_end_, sample_index, &joints);
      if (!accurate)
      {
        if (!joints.allFinite())
        {
          g_shared->edge.nonfinite.fetch_add(1);
        }
        g_shared->edge.accuracy_fail.fetch_add(1);
        g_shared->edge.accuracy_not_evaluated.fetch_add(sample_order().size() - order_index - 1);
        g_shared->edge.self_not_evaluated.fetch_add(sample_order().size());
        g_shared->edge.coupon_not_evaluated.fetch_add(sample_order().size());
        return std::make_pair(false, 0.0);
      }
      g_shared->edge.accuracy_pass.fetch_add(1);
    }
    bool self_hit = false;
    bool coupon_hit = false;
    bool error = false;
    for (std::size_t order_index = 0; order_index < sample_order().size(); ++order_index)
    {
      if (timed_out() || error || self_hit || coupon_hit)
      {
        const auto remaining = sample_order().size() - order_index;
        g_shared->edge.self_not_evaluated.fetch_add(remaining);
        g_shared->edge.coupon_not_evaluated.fetch_add(remaining);
        break;
      }
      const int sample_index = sample_order()[order_index];
      Eigen::VectorXd joints;
      sample_accuracy(
        start.values, end.values, start_position_, end_position_, start_orientation_, end_orientation_,
        t_start_, t_end_, sample_index, &joints);
      apply_collision(&g_shared->edge, joints, &self_hit, &coupon_hit, &error);
    }
    if (error || self_hit || coupon_hit || g_shared->setup_failed.load() || timed_out())
    {
      return std::make_pair(false, 0.0);
    }
    const std::pair<bool, double> cost = distance_->evaluate(start, end);
    if (!cost.first || !std::isfinite(cost.second))
    {
      g_shared->edge.nonfinite.fetch_add(1);
      return std::make_pair(false, 0.0);
    }
    g_accepted_edges.fetch_add(1);
    return cost;
  }

private:
  Eigen::Vector3d start_position_;
  Eigen::Vector3d end_position_;
  Eigen::Quaterniond start_orientation_;
  Eigen::Quaterniond end_orientation_;
  double t_start_;
  double t_end_;
  descartes_light::EdgeEvaluator<double>::ConstPtr distance_;
};

bool filter_candidate(const Candidate & candidate, const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation)
{
  g_shared->rung.evaluations.fetch_add(1);
  if (timed_out())
  {
    add_not_evaluated(&g_shared->rung, 1, false);
    return false;
  }
  if (!candidate.joints.allFinite())
  {
    g_shared->rung.nonfinite.fetch_add(1);
    add_not_evaluated(&g_shared->rung, 1, false);
    return false;
  }
  if (!within_limits(candidate.joints))
  {
    g_shared->rung.limit_fail.fetch_add(1);
    add_not_evaluated(&g_shared->rung, 1, false);
    return false;
  }
  double position_error = 0.0;
  double orientation_error = 0.0;
  if (!pose_error(candidate.joints, position, orientation, &position_error, &orientation_error) ||
      position_error > kModelPositionM || orientation_error > kModelOrientationRad)
  {
    if (!std::isfinite(position_error) || !std::isfinite(orientation_error))
    {
      g_shared->rung.nonfinite.fetch_add(1);
    }
    g_shared->rung.accuracy_fail.fetch_add(1);
    g_shared->rung.self_not_evaluated.fetch_add(1);
    g_shared->rung.coupon_not_evaluated.fetch_add(1);
    return false;
  }
  g_shared->rung.accuracy_pass.fetch_add(1);
  bool self_hit = false;
  bool coupon_hit = false;
  bool error = false;
  apply_collision(&g_shared->rung, candidate.joints, &self_hit, &coupon_hit, &error);
  return !self_hit && !coupon_hit && !error && !g_shared->setup_failed.load();
}

void filter_rung(Rung * rung, int threads)
{
  std::vector<char> keep(rung->candidates.size(), 0);
  std::atomic<std::size_t> cursor{0};
  std::atomic<std::size_t> processed{0};
  std::vector<std::thread> workers;
  const int worker_count = std::max(1, threads);
  for (int worker = 0; worker < worker_count; ++worker)
  {
    workers.emplace_back([&]() {
      while (!timed_out())
      {
        const std::size_t index = cursor.fetch_add(1);
        if (index >= rung->candidates.size())
        {
          return;
        }
        keep[index] = filter_candidate(
          rung->candidates[index], rung->position, rung->orientation) ? 1 : 0;
        processed.fetch_add(1);
      }
    });
  }
  for (std::thread & worker : workers)
  {
    worker.join();
  }
  for (std::size_t index = 0; index < keep.size(); ++index)
  {
    if (keep[index])
    {
      rung->survivors.push_back(index);
    }
  }
  if (g_shared->time_exceeded.load() && processed.load() < rung->candidates.size())
  {
    add_not_evaluated(&g_shared->rung, rung->candidates.size() - processed.load(), false);
  }
}

bool verify_threads(int threads)
{
  std::vector<std::thread> workers;
  for (int index = 0; index < threads; ++index)
  {
    workers.emplace_back([]() { local_scene(); });
  }
  for (std::thread & worker : workers)
  {
    worker.join();
  }
  std::lock_guard<std::mutex> guard(g_shared->scene_mutex);
  if (static_cast<int>(g_shared->scenes.size()) < threads || !g_shared->reference.ok)
  {
    return false;
  }
  for (const SceneInfo & info : g_shared->scenes)
  {
    if (!scenes_match(info, g_shared->reference))
    {
      return false;
    }
  }
  return !g_shared->setup_failed.load();
}

struct EdgeCase
{
  const char * name;
  bool fixture = false;
  int run_id = 0;
  int from_index = 0;
  int to_index = 0;
  Eigen::VectorXd from_joints;
  Eigen::VectorXd to_joints;
  bool expect_valid = false;
};

std::pair<bool, double> evaluate_case(
  const EdgeCase & item,
  const std::vector<std::vector<Waypoint>> & runs,
  const descartes_light::EdgeEvaluator<double>::ConstPtr & distance)
{
  Eigen::VectorXd from_joints = item.from_joints;
  Eigen::VectorXd to_joints = item.to_joints;
  const Waypoint * start = nullptr;
  const Waypoint * end = nullptr;
  if (!item.fixture)
  {
    start = &runs[static_cast<std::size_t>(item.run_id)][0];
    end = &runs[static_cast<std::size_t>(item.run_id)][1];
    from_joints = start->candidates[static_cast<std::size_t>(item.from_index)].joints;
    to_joints = end->candidates[static_cast<std::size_t>(item.to_index)].joints;
  }
  else
  {
    start = &runs[0][0];
    end = &runs[0][1];
  }
  CollisionEdgeEvaluator gate(
    start->position, end->position, start->orientation, end->orientation, 0.0, 1.0, distance);
  return gate.evaluate(
    descartes_light::State<double>(from_joints), descartes_light::State<double>(to_joints));
}

void write_counts(std::ostream & output, const char * name, const Counts & counts)
{
  output << "  \"" << name << "\": {\n"
         << "    \"evaluations\": " << counts.evaluations.load() << ",\n"
         << "    \"accuracy_pass\": " << counts.accuracy_pass.load() << ",\n"
         << "    \"accuracy_fail\": " << counts.accuracy_fail.load() << ",\n"
         << "    \"accuracy_not_evaluated\": " << counts.accuracy_not_evaluated.load() << ",\n"
         << "    \"self_clear\": " << counts.self_clear.load() << ",\n"
         << "    \"self_collision\": " << counts.self_collision.load() << ",\n"
         << "    \"self_not_evaluated\": " << counts.self_not_evaluated.load() << ",\n"
         << "    \"self_error\": " << counts.self_error.load() << ",\n"
         << "    \"coupon_clear\": " << counts.coupon_clear.load() << ",\n"
         << "    \"coupon_collision\": " << counts.coupon_collision.load() << ",\n"
         << "    \"coupon_not_evaluated\": " << counts.coupon_not_evaluated.load() << ",\n"
         << "    \"coupon_error\": " << counts.coupon_error.load() << ",\n"
         << "    \"nonfinite\": " << counts.nonfinite.load() << ",\n"
         << "    \"limit_fail\": " << counts.limit_fail.load() << "\n  },\n";
}

void write_status_file(const std::string & status, const std::string & cause, const std::string & detail)
{
  std::ofstream output(g_out_path);
  output << std::setprecision(17);
  output << "{\n  \"gate\": \"UR-ANALYTIC-IK-PROCESS-CONNECT-COLLISION\",\n"
         << "  \"status\": \"" << status << "\",\n"
         << "  \"termination_cause\": \"" << cause << "\",\n"
         << "  \"detail\": \"" << detail << "\",\n"
         << "  \"adopted\": false,\n"
         << "  \"complete\": true\n}\n";
}

bool build_rungs(
  const std::vector<Waypoint> & waypoints,
  const std::vector<Insert> * inserts,
  std::vector<Rung> * rungs)
{
  if (inserts == nullptr)
  {
    for (const Waypoint & waypoint : waypoints)
    {
      Rung rung;
      rung.sequence_id = waypoint.sequence_id;
      rung.position = waypoint.position;
      rung.orientation = waypoint.orientation;
      rung.candidates = waypoint.candidates;
      rungs->push_back(std::move(rung));
    }
    return true;
  }
  if (inserts->size() + 1 != waypoints.size())
  {
    return false;
  }
  for (std::size_t segment = 0; segment < inserts->size(); ++segment)
  {
    const Insert & insert = (*inserts)[segment];
    if (insert.from_sequence_id != waypoints[segment].sequence_id ||
        insert.to_sequence_id != waypoints[segment + 1].sequence_id)
    {
      return false;
    }
    Rung start;
    start.sequence_id = waypoints[segment].sequence_id;
    start.position = waypoints[segment].position;
    start.orientation = waypoints[segment].orientation;
    start.candidates = waypoints[segment].candidates;
    rungs->push_back(std::move(start));
    Rung mid;
    mid.inserted = true;
    mid.from_sequence_id = insert.from_sequence_id;
    mid.to_sequence_id = insert.to_sequence_id;
    mid.position = insert.position;
    mid.orientation = insert.orientation;
    mid.candidates = insert.valid;
    rungs->push_back(std::move(mid));
  }
  Rung last;
  last.sequence_id = waypoints.back().sequence_id;
  last.position = waypoints.back().position;
  last.orientation = waypoints.back().orientation;
  last.candidates = waypoints.back().candidates;
  rungs->push_back(std::move(last));
  return true;
}

struct PostStats
{
  bool within = true;
  bool self_clear = true;
  bool coupon_clear = true;
  bool error = false;
  int samples = 0;
  int coupon_skipped = 0;
  double max_position = std::numeric_limits<double>::quiet_NaN();
  double max_orientation = std::numeric_limits<double>::quiet_NaN();
};

bool postcheck(
  const std::vector<Waypoint> & waypoints,
  const std::vector<Rung> & rungs,
  const std::vector<std::size_t> & chosen,
  bool piecewise,
  PostStats * stats)
{
  const std::size_t segments = waypoints.size() - 1;
  for (std::size_t segment = 0; segment < segments; ++segment)
  {
    const Waypoint & start = waypoints[segment];
    const Waypoint & end = waypoints[segment + 1];
    const Eigen::VectorXd * start_joints = nullptr;
    const Eigen::VectorXd * mid_joints = nullptr;
    const Eigen::VectorXd * end_joints = nullptr;
    if (piecewise)
    {
      start_joints = &rungs[segment * 2].candidates[chosen[segment * 2]].joints;
      mid_joints = &rungs[segment * 2 + 1].candidates[chosen[segment * 2 + 1]].joints;
      end_joints = &rungs[segment * 2 + 2].candidates[chosen[segment * 2 + 2]].joints;
    }
    else
    {
      start_joints = &rungs[segment].candidates[chosen[segment]].joints;
      end_joints = &rungs[segment + 1].candidates[chosen[segment + 1]].joints;
    }
    for (int sample_index = 0; sample_index <= kSampleDenominator; ++sample_index)
    {
      const double parameter = static_cast<double>(sample_index) / static_cast<double>(kSampleDenominator);
      Eigen::VectorXd joints;
      if (piecewise && parameter <= 0.5)
      {
        const double local = parameter / 0.5;
        joints = (1.0 - local) * (*start_joints) + local * (*mid_joints);
      }
      else if (piecewise)
      {
        const double local = (parameter - 0.5) / 0.5;
        joints = (1.0 - local) * (*mid_joints) + local * (*end_joints);
      }
      else
      {
        joints = (1.0 - parameter) * (*start_joints) + parameter * (*end_joints);
      }
      ++stats->samples;
      const Eigen::Vector3d target = (1.0 - parameter) * start.position + parameter * end.position;
      const Eigen::Quaterniond orientation = start.orientation.slerp(parameter, end.orientation);
      double position_error = 0.0;
      double orientation_error = 0.0;
      if (!joints.allFinite() ||
          !pose_error(joints, target, orientation, &position_error, &orientation_error) ||
          position_error > kPathPositionM || orientation_error > kPathOrientationRad)
      {
        stats->within = false;
      }
      if (std::isfinite(position_error) &&
          (!std::isfinite(stats->max_position) || position_error > stats->max_position))
      {
        stats->max_position = position_error;
      }
      if (std::isfinite(orientation_error) &&
          (!std::isfinite(stats->max_orientation) || orientation_error > stats->max_orientation))
      {
        stats->max_orientation = orientation_error;
      }
      if (!joints.allFinite())
      {
        stats->error = true;
        continue;
      }
      LocalScene & local = local_scene();
      std::vector<double> query(joints.data(), joints.data() + joints.size());
      local.state.setJointGroupPositions(g_shared->group, query);
      local.state.update();
      const Probe self = probe(true, local.state);
      if (self == Probe::ERROR)
      {
        stats->error = true;
        stats->self_clear = false;
        ++stats->coupon_skipped;
        continue;
      }
      if (self == Probe::HIT)
      {
        stats->self_clear = false;
        ++stats->coupon_skipped;
        continue;
      }
      const Probe coupon = probe(false, local.state);
      if (coupon == Probe::ERROR)
      {
        stats->error = true;
        stats->coupon_clear = false;
      }
      else if (coupon == Probe::HIT)
      {
        stats->coupon_clear = false;
      }
    }
  }
  return stats->within && stats->self_clear && stats->coupon_clear && !stats->error &&
    stats->coupon_skipped == 0;
}

int precheck(const std::vector<std::vector<Waypoint>> & runs, int threads)
{
  if (!verify_threads(threads))
  {
    write_status_file("EXECUTION_ERROR", "SETUP", "scene_mismatch");
    return 4;
  }
  auto distance = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<double>>();
  std::vector<EdgeCase> cases(4);
  cases[0].name = "run0_adopted_edge";
  cases[0].run_id = 0;
  cases[0].from_index = 160;
  cases[0].to_index = 160;
  cases[0].expect_valid = true;
  cases[1].name = "run1_recorded_collision_edge";
  cases[1].run_id = 1;
  cases[1].from_index = 192;
  cases[1].to_index = 192;
  cases[1].expect_valid = false;
  cases[2].name = "nonfinite_fixture";
  cases[2].fixture = true;
  cases[2].from_joints = Eigen::VectorXd::Constant(6, std::numeric_limits<double>::quiet_NaN());
  cases[2].to_joints = Eigen::VectorXd::Zero(6);
  cases[2].expect_valid = false;
  cases[3].name = "limit_fixture";
  cases[3].fixture = true;
  cases[3].from_joints = Eigen::VectorXd::Constant(6, 100.0);
  cases[3].to_joints = Eigen::VectorXd::Zero(6);
  cases[3].expect_valid = false;
  std::vector<std::pair<bool, double>> single(cases.size());
  std::vector<std::pair<bool, double>> parallel(cases.size());
  for (std::size_t index = 0; index < cases.size(); ++index)
  {
    single[index] = evaluate_case(cases[index], runs, distance);
  }
  std::vector<std::thread> workers;
  for (std::size_t index = 0; index < cases.size(); ++index)
  {
    workers.emplace_back([&, index]() {
      parallel[index] = evaluate_case(cases[index], runs, distance);
    });
  }
  for (std::thread & worker : workers)
  {
    worker.join();
  }
  bool matched = true;
  bool valid_included = false;
  bool invalid_included = false;
  for (std::size_t index = 0; index < cases.size(); ++index)
  {
    if (single[index].first != parallel[index].first ||
        (single[index].first && single[index].second != parallel[index].second))
    {
      matched = false;
    }
    if (single[index].first)
    {
      valid_included = true;
    }
    else
    {
      invalid_included = true;
    }
    if (single[index].first != cases[index].expect_valid)
    {
      matched = false;
    }
  }
  if (g_shared->setup_failed.load())
  {
    matched = false;
  }
  std::ofstream output(g_out_path);
  output << std::setprecision(17);
  output << "{\n  \"gate\": \"UR-ANALYTIC-IK-PROCESS-CONNECT-COLLISION\",\n"
         << "  \"mode\": \"precheck\",\n"
         << "  \"status\": \"" << (matched && valid_included && invalid_included ? "PRECHECK_PASS" : "EXECUTION_ERROR")
         << "\",\n  \"termination_cause\": \"COMPLETED\",\n"
         << "  \"thread_verdict_match\": " << (matched ? "true" : "false") << ",\n"
         << "  \"valid_case_included\": " << (valid_included ? "true" : "false") << ",\n"
         << "  \"invalid_case_included\": " << (invalid_included ? "true" : "false") << ",\n"
         << "  \"scene_count\": " << g_shared->scenes.size() << ",\n"
         << "  \"cases\": [\n";
  for (std::size_t index = 0; index < cases.size(); ++index)
  {
    if (index)
    {
      output << ",\n";
    }
    output << "    {\"name\": \"" << cases[index].name
           << "\", \"expected_valid\": " << (cases[index].expect_valid ? "true" : "false")
           << ", \"single_valid\": " << (single[index].first ? "true" : "false")
           << ", \"multi_valid\": " << (parallel[index].first ? "true" : "false")
           << ", \"single_cost\": ";
    write_number(output, single[index].second);
    output << ", \"multi_cost\": ";
    write_number(output, parallel[index].second);
    output << "}";
  }
  output << "\n  ],\n  \"complete\": true\n}\n";
  std::cout << "PRECHECK " << (matched && valid_included && invalid_included ? "PASS" : "FAIL") << "\n";
  return matched && valid_included && invalid_included ? 0 : 5;
}

int search_run(
  const std::vector<Waypoint> & waypoints,
  const std::vector<Insert> * inserts,
  int run_id,
  int threads)
{
  if (!verify_threads(threads))
  {
    write_status_file("EXECUTION_ERROR", "SETUP", "scene_mismatch");
    return 4;
  }
  std::vector<Rung> rungs;
  if (!build_rungs(waypoints, inserts, &rungs))
  {
    write_status_file("EXECUTION_ERROR", "SETUP", "rung_inputs");
    return 4;
  }
  bool missing = false;
  for (Rung & rung : rungs)
  {
    filter_rung(&rung, threads);
    if (rung.survivors.empty())
    {
      missing = true;
    }
  }
  std::string status = "NO_CONNECTING_EDGE";
  std::string cause = "COMPLETED";
  if (g_shared->time_exceeded.load())
  {
    status = "TIME_OR_RESOURCE_LIMIT";
    cause = "WALL";
    missing = false;
  }
  else if (g_shared->setup_failed.load() ||
           g_shared->rung.self_error.load() > 0 || g_shared->rung.coupon_error.load() > 0)
  {
    status = "EXECUTION_ERROR";
    cause = "SETUP";
  }
  else if (missing)
  {
    status = "NO_COLLISION_FREE_CANDIDATE";
  }
  double ladder_cost = std::numeric_limits<double>::quiet_NaN();
  std::vector<std::size_t> chosen;
  std::uint64_t edge_ceiling = 0;
  bool ceiling_overflow = false;
  for (std::size_t index = 0; index + 1 < rungs.size(); ++index)
  {
    const std::uint64_t left = rungs[index].survivors.size();
    const std::uint64_t right = rungs[index + 1].survivors.size();
    std::uint64_t product = 0;
    if (left != 0 && right > std::numeric_limits<std::uint64_t>::max() / left)
    {
      ceiling_overflow = true;
    }
    else
    {
      product = left * right;
    }
    if (edge_ceiling > std::numeric_limits<std::uint64_t>::max() - product)
    {
      ceiling_overflow = true;
    }
    else
    {
      edge_ceiling += product;
    }
  }
  PostStats stats;
  bool have_path = false;
  if (status == "NO_CONNECTING_EDGE")
  {
    std::vector<std::vector<descartes_light::State<double>::ConstPtr>> states(rungs.size());
    std::vector<descartes_light::WaypointSampler<double>::ConstPtr> samplers;
    std::vector<descartes_light::EdgeEvaluator<double>::ConstPtr> gates;
    auto distance = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<double>>();
    for (std::size_t rung_index = 0; rung_index < rungs.size(); ++rung_index)
    {
      for (std::size_t survivor : rungs[rung_index].survivors)
      {
        states[rung_index].push_back(std::make_shared<descartes_light::State<double>>(
          rungs[rung_index].candidates[survivor].joints));
      }
      samplers.push_back(std::make_shared<RecordedJointSampler>(states[rung_index]));
    }
    for (std::size_t segment = 0; segment + 1 < waypoints.size(); ++segment)
    {
      const double spans[2][2] = {{0.0, inserts ? 0.5 : 1.0}, {0.5, 1.0}};
      const int half_count = inserts ? 2 : 1;
      for (int half = 0; half < half_count; ++half)
      {
        gates.push_back(std::make_shared<CollisionEdgeEvaluator>(
          waypoints[segment].position, waypoints[segment + 1].position,
          waypoints[segment].orientation, waypoints[segment + 1].orientation,
          spans[half][0], spans[half][1], distance));
      }
    }
    if (gates.size() + 1 != rungs.size())
    {
      write_status_file("EXECUTION_ERROR", "SETUP", "edge_count");
      return 4;
    }
    try
    {
      descartes_light::LadderGraphSolver<double> ladder(threads);
      const auto built = ladder.build(samplers, gates, {});
      if (g_shared->time_exceeded.load())
      {
        status = "TIME_OR_RESOURCE_LIMIT";
        cause = "WALL";
      }
      else if (g_shared->setup_failed.load() || g_shared->edge.self_error.load() > 0 ||
               g_shared->edge.coupon_error.load() > 0)
      {
        status = "EXECUTION_ERROR";
        cause = "SETUP";
      }
      else if (built)
      {
        try
        {
          const descartes_light::SearchResult<double> result = ladder.search();
          if (result.trajectory.size() != rungs.size() || !std::isfinite(result.cost))
          {
            status = "EXECUTION_ERROR";
            cause = "EXCEPTION";
          }
          else
          {
            chosen.resize(rungs.size());
            for (std::size_t rung_index = 0; rung_index < rungs.size(); ++rung_index)
            {
              const auto match = std::find(
                states[rung_index].begin(), states[rung_index].end(), result.trajectory[rung_index]);
              if (match == states[rung_index].end() ||
                  !same_vector(
                    rungs[rung_index].candidates[rungs[rung_index].survivors[static_cast<std::size_t>(
                      match - states[rung_index].begin())]].joints,
                    result.trajectory[rung_index]->values))
              {
                status = "EXECUTION_ERROR";
                cause = "EXCEPTION";
                break;
              }
              chosen[rung_index] = rungs[rung_index].survivors[static_cast<std::size_t>(
                match - states[rung_index].begin())];
            }
            if (status == "NO_CONNECTING_EDGE")
            {
              ladder_cost = result.cost;
              have_path = true;
            }
          }
        }
        catch (const std::runtime_error &)
        {
          have_path = false;
        }
      }
    }
    catch (const std::bad_alloc &)
    {
      status = "TIME_OR_RESOURCE_LIMIT";
      cause = "MEMORY";
    }
    catch (const std::exception &)
    {
      status = "EXECUTION_ERROR";
      cause = "EXCEPTION";
    }
  }
  bool adopted = false;
  if (have_path && status == "NO_CONNECTING_EDGE")
  {
    adopted = postcheck(waypoints, rungs, chosen, inserts != nullptr, &stats);
    status = adopted ? "ADOPTED" : "POSTCHECK_FAIL";
  }
  if (g_shared->time_exceeded.load() && status != "TIME_OR_RESOURCE_LIMIT")
  {
    status = "TIME_OR_RESOURCE_LIMIT";
    cause = "WALL";
    adopted = false;
  }
  std::ofstream output(g_out_path);
  output << std::setprecision(17);
  output << "{\n  \"gate\": \"UR-ANALYTIC-IK-PROCESS-CONNECT-COLLISION\",\n"
         << "  \"run_id\": " << run_id << ",\n"
         << "  \"status\": \"" << status << "\",\n"
         << "  \"termination_cause\": \"" << cause << "\",\n"
         << "  \"adopted\": " << (adopted ? "true" : "false") << ",\n"
         << "  \"piecewise_stored_midpoint\": " << (inserts != nullptr ? "true" : "false") << ",\n"
         << "  \"threads\": " << threads << ",\n"
         << "  \"scene_count\": " << g_shared->scenes.size() << ",\n"
         << "  \"edge_ceiling\": " << edge_ceiling << ",\n"
         << "  \"edge_ceiling_overflow\": " << (ceiling_overflow ? "true" : "false") << ",\n";
  write_counts(output, "rung_filter", g_shared->rung);
  write_counts(output, "edge_check", g_shared->edge);
  output << "  \"postcheck_samples\": " << stats.samples << ",\n"
         << "  \"postcheck_accuracy_pass\": " << (stats.within ? "true" : "false") << ",\n"
         << "  \"postcheck_self_clear\": " << (stats.self_clear ? "true" : "false") << ",\n"
         << "  \"postcheck_coupon_clear\": " << (stats.coupon_clear ? "true" : "false") << ",\n"
         << "  \"postcheck_coupon_not_evaluated\": " << stats.coupon_skipped << ",\n"
         << "  \"postcheck_error\": " << (stats.error ? "true" : "false") << ",\n"
         << "  \"ladder_cost\": ";
  write_number(output, ladder_cost);
  output << ",\n  \"max_position_error_m\": ";
  write_number(output, stats.max_position);
  output << ",\n  \"max_orientation_error_rad\": ";
  write_number(output, stats.max_orientation);
  output << ",\n  \"survivor_counts\": [";
  for (std::size_t index = 0; index < rungs.size(); ++index)
  {
    if (index)
    {
      output << ", ";
    }
    output << rungs[index].survivors.size();
  }
  output << "],\n  \"selected\": [";
  if (have_path)
  {
    for (std::size_t index = 0; index < chosen.size(); ++index)
    {
      if (index)
      {
        output << ", ";
      }
      const Candidate & candidate = rungs[index].candidates[chosen[index]];
      output << "{\"candidate_index\": " << chosen[index] << ", \"joints_rad\": [";
      for (int joint_index = 0; joint_index < 6; ++joint_index)
      {
        if (joint_index)
        {
          output << ", ";
        }
        write_number(output, candidate.joints[joint_index]);
      }
      output << "]}";
    }
  }
  output << "],\n  \"volume_containment\": \"NOT_CHECKED\",\n"
         << "  \"between_sample_guarantee\": false,\n"
         << "  \"tool_fixture_table\": \"NOT_EVALUATED\",\n"
         << "  \"physical_execution\": \"NOT_CLAIMED\",\n"
         << "  \"complete\": true\n}\n";
  std::cout << "RUN " << run_id << " " << status << " " << cause << "\n";
  return status == "EXECUTION_ERROR" ? 4 : 0;
}

struct LadderOutcome
{
  std::string status = "NOT_STARTED_PRIOR";
  double cost = std::numeric_limits<double>::quiet_NaN();
  std::vector<std::size_t> chosen;
  std::uint64_t ceiling = 0;
  std::uint64_t evaluations = 0;
  bool ceiling_overflow = false;
  double elapsed_s = 0.0;
};

struct SampleJudge
{
  bool accuracy = true;
  bool limits = true;
  bool self_clear = true;
  bool coupon_clear = true;
  bool junction = true;
  bool error = false;
  bool accuracy_applied = true;
  int samples = 0;
  int accuracy_fail = 0;
  int limit_fail = 0;
  int self_hit = 0;
  int coupon_hit = 0;
  int collision_not_evaluated = 0;
  double max_position = std::numeric_limits<double>::quiet_NaN();
  double max_orientation = std::numeric_limits<double>::quiet_NaN();
  double junction_rad = 0.0;
  std::string status = "NOT_STARTED_PRIOR";
  double elapsed_s = 0.0;
};

double raw_difference(const Eigen::VectorXd & left, const Eigen::VectorXd & right)
{
  if (left.size() != right.size() || left.size() == 0)
  {
    return std::numeric_limits<double>::infinity();
  }
  double maximum = 0.0;
  for (int index = 0; index < left.size(); ++index)
  {
    if (!std::isfinite(left[index]) || !std::isfinite(right[index]))
    {
      return std::numeric_limits<double>::infinity();
    }
    maximum = std::max(maximum, std::abs(left[index] - right[index]));
  }
  return maximum;
}

double max_joint_span()
{
  double span = 0.0;
  for (int index = 0; index < g_shared->limits.rows(); ++index)
  {
    span = std::max(span, g_shared->limits(index, 1) - g_shared->limits(index, 0));
  }
  return span;
}

void zero_edge_counts()
{
  Counts & counts = g_shared->edge;
  counts.accuracy_pass = 0;
  counts.accuracy_fail = 0;
  counts.accuracy_not_evaluated = 0;
  counts.self_clear = 0;
  counts.self_collision = 0;
  counts.self_not_evaluated = 0;
  counts.self_error = 0;
  counts.coupon_clear = 0;
  counts.coupon_collision = 0;
  counts.coupon_not_evaluated = 0;
  counts.coupon_error = 0;
  counts.nonfinite = 0;
  counts.limit_fail = 0;
  counts.evaluations = 0;
}

class LinearJointEdgeEvaluator : public descartes_light::EdgeEvaluator<double>
{
public:
  LinearJointEdgeEvaluator(double step, descartes_light::EdgeEvaluator<double>::ConstPtr distance)
  : step_(step), distance_(std::move(distance))
  {
  }

  std::pair<bool, double> evaluate(
    const descartes_light::State<double> & start,
    const descartes_light::State<double> & end) const override
  {
    g_shared->edge.evaluations.fetch_add(1);
    if (timed_out() || g_shared->setup_failed.load())
    {
      return std::make_pair(false, 0.0);
    }
    if (!start.values.allFinite() || !end.values.allFinite())
    {
      g_shared->edge.nonfinite.fetch_add(1);
      return std::make_pair(false, 0.0);
    }
    const double delta = raw_difference(start.values, end.values);
    int splits = 1;
    if (!std::isfinite(delta))
    {
      g_shared->edge.nonfinite.fetch_add(1);
      return std::make_pair(false, 0.0);
    }
    if (delta > step_)
    {
      splits = static_cast<int>(std::ceil((delta / step_) - 1.0e-12));
      if (splits < 1)
      {
        splits = 1;
      }
    }
    for (int sample = 0; sample <= splits; ++sample)
    {
      if (timed_out())
      {
        return std::make_pair(false, 0.0);
      }
      const double ratio = static_cast<double>(sample) / static_cast<double>(splits);
      const Eigen::VectorXd joints = (1.0 - ratio) * start.values + ratio * end.values;
      if (!joints.allFinite())
      {
        g_shared->edge.nonfinite.fetch_add(1);
        return std::make_pair(false, 0.0);
      }
      if (!within_limits(joints))
      {
        g_shared->edge.limit_fail.fetch_add(1);
        return std::make_pair(false, 0.0);
      }
      bool self_hit = false;
      bool coupon_hit = false;
      bool error = false;
      apply_collision(&g_shared->edge, joints, &self_hit, &coupon_hit, &error);
      if (error || g_shared->setup_failed.load())
      {
        return std::make_pair(false, 0.0);
      }
      if (self_hit || coupon_hit)
      {
        return std::make_pair(false, 0.0);
      }
    }
    const std::pair<bool, double> cost = distance_->evaluate(start, end);
    if (!cost.first || !std::isfinite(cost.second))
    {
      g_shared->edge.nonfinite.fetch_add(1);
      return std::make_pair(false, 0.0);
    }
    g_accepted_edges.fetch_add(1);
    return cost;
  }

private:
  double step_;
  descartes_light::EdgeEvaluator<double>::ConstPtr distance_;
};

bool load_transition_waypoints(const std::string & path, Waypoint * first, Waypoint * second)
{
  std::ifstream input(path, std::ios::binary);
  char magic[8] = {};
  input.read(magic, 8);
  std::uint32_t count = 0;
  if (!input || std::string(magic, 4) != "TRN1" || !read_pod(input, &count) || count != 20)
  {
    return false;
  }
  for (int index = 0; index < 20; ++index)
  {
    std::int32_t stored_index = 0;
    std::int32_t from_run = 0;
    std::int32_t to_run = 0;
    std::int32_t sequence_id = 0;
    if (!read_pod(input, &stored_index) || !read_pod(input, &from_run) || !read_pod(input, &to_run) ||
        !read_pod(input, &sequence_id) || stored_index != index)
    {
      return false;
    }
    double ignored_from[6];
    double ignored_to[6];
    double position[3];
    double quaternion[4];
    input.read(reinterpret_cast<char *>(ignored_from), sizeof(ignored_from));
    input.read(reinterpret_cast<char *>(ignored_to), sizeof(ignored_to));
    input.read(reinterpret_cast<char *>(position), sizeof(position));
    input.read(reinterpret_cast<char *>(quaternion), sizeof(quaternion));
    std::uint32_t candidate_count = 0;
    if (!read_pod(input, &candidate_count) || candidate_count != 256)
    {
      return false;
    }
    Waypoint * target = nullptr;
    if (index == 0 && sequence_id == 35)
    {
      target = first;
    }
    else if (index == 1 && sequence_id == 70)
    {
      target = second;
    }
    if (target != nullptr)
    {
      target->sequence_id = sequence_id;
      target->position = Eigen::Vector3d(position[0], position[1], position[2]);
      target->orientation = Eigen::Quaterniond(quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
      target->orientation.normalize();
      target->candidates.resize(256);
    }
    for (int candidate_index = 0; candidate_index < 256; ++candidate_index)
    {
      std::uint8_t source = 0;
      std::int32_t class_id = 0;
      if (!read_pod(input, &source) || !read_pod(input, &class_id) || (source != 0 && source != 1))
      {
        return false;
      }
      double joints[6];
      input.read(reinterpret_cast<char *>(joints), sizeof(joints));
      if (target != nullptr)
      {
        Candidate & candidate = target->candidates[static_cast<std::size_t>(candidate_index)];
        candidate.source_code = source;
        candidate.kinematic_class_id = class_id;
        candidate.joints = Eigen::VectorXd(6);
        for (int joint = 0; joint < 6; ++joint)
        {
          if (!std::isfinite(joints[joint]))
          {
            return false;
          }
          candidate.joints[joint] = joints[joint];
        }
      }
    }
  }
  return static_cast<bool>(input) && input.peek() == EOF && first->sequence_id == 35 && second->sequence_id == 70;
}

bool pose_matches(
  const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation,
  const double expected_position[3], const double expected_xyzw[4])
{
  for (int index = 0; index < 3; ++index)
  {
    if (!(std::abs(position[index] - expected_position[index]) <= 1.0e-12))
    {
      return false;
    }
  }
  Eigen::Quaterniond expected(expected_xyzw[3], expected_xyzw[0], expected_xyzw[1], expected_xyzw[2]);
  expected.normalize();
  Eigen::Quaterniond actual = orientation.normalized();
  if (actual.dot(expected) < 0.0)
  {
    expected = Eigen::Quaterniond(-expected.w(), -expected.x(), -expected.y(), -expected.z());
  }
  return std::abs(actual.x() - expected.x()) <= 1.0e-12 &&
    std::abs(actual.y() - expected.y()) <= 1.0e-12 &&
    std::abs(actual.z() - expected.z()) <= 1.0e-12 &&
    std::abs(actual.w() - expected.w()) <= 1.0e-12;
}

bool consecutive_run(const std::vector<Waypoint> & run, int first, int last)
{
  if (static_cast<int>(run.size()) != last - first + 1 || run.front().sequence_id != first ||
      run.back().sequence_id != last)
  {
    return false;
  }
  for (int index = 0; index < static_cast<int>(run.size()); ++index)
  {
    if (run[static_cast<std::size_t>(index)].sequence_id != first + index ||
        run[static_cast<std::size_t>(index)].candidates.size() != 256)
    {
      return false;
    }
  }
  return true;
}

LadderOutcome run_ladder(const std::vector<Rung> & rungs, bool linear_first, int threads)
{
  LadderOutcome outcome;
  const double started = process_elapsed();
  bool missing = false;
  for (std::size_t index = 0; index + 1 < rungs.size(); ++index)
  {
    const std::uint64_t left = rungs[index].survivors.size();
    const std::uint64_t right = rungs[index + 1].survivors.size();
    if (left == 0 || right == 0)
    {
      missing = true;
    }
    if (left != 0 && right > std::numeric_limits<std::uint64_t>::max() / left)
    {
      outcome.ceiling_overflow = true;
    }
    else if (outcome.ceiling > std::numeric_limits<std::uint64_t>::max() - left * right)
    {
      outcome.ceiling_overflow = true;
    }
    else
    {
      outcome.ceiling += left * right;
    }
  }
  if (outcome.ceiling_overflow)
  {
    outcome.status = "EXECUTION_ERROR";
    outcome.elapsed_s = process_elapsed() - started;
    return outcome;
  }
  if (missing)
  {
    outcome.status = g_shared->time_exceeded.load() ? "GRAPH_INCOMPLETE" : "NO_PATH_IN_COMPLETED_GRAPH";
    outcome.elapsed_s = process_elapsed() - started;
    return outcome;
  }
  zero_edge_counts();
  std::vector<std::vector<descartes_light::State<double>::ConstPtr>> states(rungs.size());
  std::vector<descartes_light::WaypointSampler<double>::ConstPtr> samplers;
  std::vector<descartes_light::EdgeEvaluator<double>::ConstPtr> gates;
  auto distance = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<double>>();
  for (std::size_t rung_index = 0; rung_index < rungs.size(); ++rung_index)
  {
    for (std::size_t survivor : rungs[rung_index].survivors)
    {
      states[rung_index].push_back(std::make_shared<descartes_light::State<double>>(
        rungs[rung_index].candidates[survivor].joints));
    }
    samplers.push_back(std::make_shared<RecordedJointSampler>(states[rung_index]));
  }
  const double step = 0.01 * max_joint_span();
  for (std::size_t segment = 0; segment + 1 < rungs.size(); ++segment)
  {
    if (linear_first && segment == 0)
    {
      gates.push_back(std::make_shared<LinearJointEdgeEvaluator>(step, distance));
    }
    else
    {
      gates.push_back(std::make_shared<CollisionEdgeEvaluator>(
        rungs[segment].position, rungs[segment + 1].position,
        rungs[segment].orientation, rungs[segment + 1].orientation,
        0.0, 1.0, distance));
    }
  }
  bool have_path = false;
  try
  {
    descartes_light::LadderGraphSolver<double> ladder(threads);
    const auto built = ladder.build(samplers, gates, {});
    outcome.evaluations = g_shared->edge.evaluations.load();
    if (g_shared->time_exceeded.load())
    {
      outcome.status = "GRAPH_INCOMPLETE";
    }
    else if (g_shared->setup_failed.load() || g_shared->edge.self_error.load() > 0 ||
             g_shared->edge.coupon_error.load() > 0)
    {
      outcome.status = "EXECUTION_ERROR";
    }
    else if (outcome.evaluations != outcome.ceiling)
    {
      outcome.status = "GRAPH_INCOMPLETE";
    }
    else if (!built)
    {
      outcome.status = "NO_PATH_IN_COMPLETED_GRAPH";
    }
    else
    {
      const descartes_light::SearchResult<double> result = ladder.search();
      if (result.trajectory.size() != rungs.size() || !std::isfinite(result.cost))
      {
        outcome.status = "EXECUTION_ERROR";
      }
      else
      {
        outcome.chosen.resize(rungs.size());
        bool matched = true;
        for (std::size_t rung_index = 0; rung_index < rungs.size(); ++rung_index)
        {
          const auto found = std::find(
            states[rung_index].begin(), states[rung_index].end(), result.trajectory[rung_index]);
          if (found == states[rung_index].end())
          {
            matched = false;
            break;
          }
          outcome.chosen[rung_index] = rungs[rung_index].survivors[static_cast<std::size_t>(
            found - states[rung_index].begin())];
        }
        if (!matched)
        {
          outcome.status = "EXECUTION_ERROR";
        }
        else if (g_shared->time_exceeded.load())
        {
          outcome.status = "GRAPH_INCOMPLETE";
        }
        else
        {
          outcome.status = "PATH";
          outcome.cost = result.cost;
          have_path = true;
        }
      }
    }
  }
  catch (const std::bad_alloc &)
  {
    outcome.status = "GRAPH_INCOMPLETE";
  }
  catch (const std::runtime_error &)
  {
    outcome.evaluations = g_shared->edge.evaluations.load();
    outcome.status = outcome.evaluations == outcome.ceiling && !g_shared->time_exceeded.load() ?
      "NO_PATH_IN_COMPLETED_GRAPH" : "GRAPH_INCOMPLETE";
  }
  catch (const std::exception &)
  {
    outcome.status = "EXECUTION_ERROR";
  }
  if (!have_path)
  {
    outcome.chosen.clear();
  }
  outcome.elapsed_s = process_elapsed() - started;
  return outcome;
}

void note_sample_error(SampleJudge * judge, double position_error, double orientation_error)
{
  if (std::isfinite(position_error) &&
      (!std::isfinite(judge->max_position) || position_error > judge->max_position))
  {
    judge->max_position = position_error;
  }
  if (std::isfinite(orientation_error) &&
      (!std::isfinite(judge->max_orientation) || orientation_error > judge->max_orientation))
  {
    judge->max_orientation = orientation_error;
  }
}

SampleJudge judge_process_path(const std::vector<Rung> & rungs, const std::vector<std::size_t> & chosen)
{
  SampleJudge judge;
  if (chosen.size() != rungs.size() || rungs.size() < 2)
  {
    judge.status = "EXECUTION_ERROR";
    judge.error = true;
    return judge;
  }
  for (std::size_t segment = 0; segment + 1 < rungs.size(); ++segment)
  {
    const Eigen::VectorXd & start = rungs[segment].candidates[chosen[segment]].joints;
    const Eigen::VectorXd & end = rungs[segment + 1].candidates[chosen[segment + 1]].joints;
    for (int sample_index = 0; sample_index <= kSampleDenominator; ++sample_index)
    {
      if (timed_out())
      {
        judge.status = "GRAPH_INCOMPLETE";
        judge.error = true;
        return judge;
      }
      const double parameter = static_cast<double>(sample_index) / static_cast<double>(kSampleDenominator);
      const Eigen::VectorXd joints = (1.0 - parameter) * start + parameter * end;
      ++judge.samples;
      if (sample_index == 0 || sample_index == kSampleDenominator)
      {
        const Eigen::VectorXd & endpoint = sample_index == 0 ? start : end;
        judge.junction_rad = std::max(judge.junction_rad, raw_difference(joints, endpoint));
        if (!(judge.junction_rad <= 1.0e-12))
        {
          judge.junction = false;
        }
      }
      if (!joints.allFinite() || !within_limits(joints))
      {
        judge.limits = false;
        ++judge.limit_fail;
        ++judge.collision_not_evaluated;
        continue;
      }
      double position_error = 0.0;
      double orientation_error = 0.0;
      const Eigen::Vector3d target = (1.0 - parameter) * rungs[segment].position + parameter * rungs[segment + 1].position;
      const Eigen::Quaterniond orientation = rungs[segment].orientation.slerp(parameter, rungs[segment + 1].orientation);
      const bool accurate = pose_error(joints, target, orientation, &position_error, &orientation_error) &&
        position_error <= kPathPositionM && orientation_error <= kPathOrientationRad;
      note_sample_error(&judge, position_error, orientation_error);
      if (!accurate)
      {
        judge.accuracy = false;
        ++judge.accuracy_fail;
        ++judge.collision_not_evaluated;
        continue;
      }
      bool self_hit = false;
      bool coupon_hit = false;
      bool error = false;
      apply_collision(&g_shared->edge, joints, &self_hit, &coupon_hit, &error);
      if (error)
      {
        judge.error = true;
        judge.self_clear = false;
        ++judge.collision_not_evaluated;
        continue;
      }
      if (self_hit)
      {
        judge.self_clear = false;
        ++judge.self_hit;
        ++judge.collision_not_evaluated;
        continue;
      }
      if (coupon_hit)
      {
        judge.coupon_clear = false;
        ++judge.coupon_hit;
      }
    }
  }
  judge.status = judge.accuracy && judge.limits && judge.self_clear && judge.coupon_clear && judge.junction &&
    !judge.error && judge.collision_not_evaluated == 0 && judge.samples > 0 ? "POSTCHECK_PASS" : "POSTCHECK_FAIL";
  return judge;
}

SampleJudge judge_linear_path(const Eigen::VectorXd & start, const Eigen::VectorXd & end)
{
  SampleJudge judge;
  judge.accuracy_applied = false;
  const double step = 0.01 * max_joint_span();
  const double delta = raw_difference(start, end);
  int splits = 1;
  if (!std::isfinite(delta))
  {
    judge.error = true;
    judge.status = "POSTCHECK_FAIL";
    return judge;
  }
  if (delta > step)
  {
    splits = static_cast<int>(std::ceil((delta / step) - 1.0e-12));
    if (splits < 1)
    {
      splits = 1;
    }
  }
  for (int sample = 0; sample <= splits; ++sample)
  {
    if (timed_out())
    {
      judge.status = "GRAPH_INCOMPLETE";
      judge.error = true;
      return judge;
    }
    const double ratio = static_cast<double>(sample) / static_cast<double>(splits);
    const Eigen::VectorXd joints = (1.0 - ratio) * start + ratio * end;
    ++judge.samples;
    if (sample == 0 || sample == splits)
    {
      const Eigen::VectorXd & endpoint = sample == 0 ? start : end;
      judge.junction_rad = std::max(judge.junction_rad, raw_difference(joints, endpoint));
      if (!(judge.junction_rad <= 1.0e-12))
      {
        judge.junction = false;
      }
    }
    if (!joints.allFinite() || !within_limits(joints))
    {
      judge.limits = false;
      ++judge.limit_fail;
      ++judge.collision_not_evaluated;
      continue;
    }
    bool self_hit = false;
    bool coupon_hit = false;
    bool error = false;
    apply_collision(&g_shared->edge, joints, &self_hit, &coupon_hit, &error);
    if (error)
    {
      judge.error = true;
      ++judge.collision_not_evaluated;
      continue;
    }
    if (self_hit)
    {
      judge.self_clear = false;
      ++judge.self_hit;
      ++judge.collision_not_evaluated;
      continue;
    }
    if (coupon_hit)
    {
      judge.coupon_clear = false;
      ++judge.coupon_hit;
    }
  }
  judge.status = judge.limits && judge.self_clear && judge.coupon_clear && judge.junction && !judge.error &&
    judge.collision_not_evaluated == 0 && judge.samples > 0 ? "POSTCHECK_PASS" : "POSTCHECK_FAIL";
  return judge;
}

void write_judge(std::ostream & output, const SampleJudge & judge)
{
  output << "{\"status\": \"" << judge.status
         << "\", \"accuracy_applied\": " << (judge.accuracy_applied ? "true" : "false")
         << ", \"accuracy_pass\": " << (judge.accuracy ? "true" : "false")
         << ", \"limits_pass\": " << (judge.limits ? "true" : "false")
         << ", \"self_clear\": " << (judge.self_clear ? "true" : "false")
         << ", \"coupon_clear\": " << (judge.coupon_clear ? "true" : "false")
         << ", \"junction_pass\": " << (judge.junction ? "true" : "false")
         << ", \"error\": " << (judge.error ? "true" : "false")
         << ", \"samples\": " << judge.samples
         << ", \"accuracy_fail\": " << judge.accuracy_fail
         << ", \"limit_fail\": " << judge.limit_fail
         << ", \"self_hit\": " << judge.self_hit
         << ", \"coupon_hit\": " << judge.coupon_hit
         << ", \"collision_not_evaluated\": " << judge.collision_not_evaluated
         << ", \"junction_rad\": ";
  write_number(output, judge.junction_rad);
  output << ", \"max_position_error_m\": ";
  write_number(output, judge.max_position);
  output << ", \"max_orientation_error_rad\": ";
  write_number(output, judge.max_orientation);
  output << ", \"elapsed_s\": ";
  write_number(output, judge.elapsed_s);
  output << "}";
}

void write_choice(std::ostream & output, const std::vector<Rung> & rungs, const std::vector<std::size_t> & chosen)
{
  output << "[";
  for (std::size_t index = 0; index < chosen.size(); ++index)
  {
    if (index)
    {
      output << ", ";
    }
    const Candidate & candidate = rungs[index].candidates[chosen[index]];
    output << "{\"sequence_id\": " << rungs[index].sequence_id
           << ", \"candidate_index\": " << chosen[index]
           << ", \"joints_rad\": [";
    for (int joint = 0; joint < candidate.joints.size(); ++joint)
    {
      if (joint)
      {
        output << ", ";
      }
      write_number(output, candidate.joints[joint]);
    }
    output << "]}";
  }
  output << "]";
}

int run_boundary(const std::vector<std::vector<Waypoint>> & runs, const std::string & transitions_path, int threads)
{
  const double expected_35_position[3] = {0.49252740483719998, -0.044999999908300004, 0.15933381987055001};
  const double expected_35_xyzw[4] = {0.99999723809999996, 0.0, -0.002350286, 0.0};
  const double expected_70_position[3] = {0.32443337953989998, -0.039877029329600008, 0.15946830394115};
  const double expected_70_xyzw[4] = {0.99999527440000002, 1.0999999999999999e-09, 3.5289999999999999e-07, 0.0030742839};
  Waypoint sequence_35;
  Waypoint sequence_70;
  std::string setup_status = "PASS";
  if (!verify_threads(threads) || !consecutive_run(runs[1], 36, 69) || !consecutive_run(runs[2], 71, 104) ||
      !load_transition_waypoints(transitions_path, &sequence_35, &sequence_70) ||
      !pose_matches(sequence_35.position, sequence_35.orientation, expected_35_position, expected_35_xyzw) ||
      !pose_matches(sequence_70.position, sequence_70.orientation, expected_70_position, expected_70_xyzw))
  {
    setup_status = "EXECUTION_ERROR";
  }
  std::vector<Rung> pass1;
  std::vector<Rung> pass2;
  if (setup_status == "PASS")
  {
    std::vector<Waypoint> pass1_points;
    pass1_points.push_back(sequence_35);
    pass1_points.insert(pass1_points.end(), runs[1].begin(), runs[1].end());
    std::vector<Waypoint> pass2_points;
    pass2_points.push_back(sequence_70);
    pass2_points.insert(pass2_points.end(), runs[2].begin(), runs[2].end());
    build_rungs(pass1_points, nullptr, &pass1);
    build_rungs(pass2_points, nullptr, &pass2);
  }
  std::string filter_status = "NOT_STARTED_PRIOR";
  double filter_elapsed = 0.0;
  if (setup_status == "PASS")
  {
    if (!begin_phase(kFilterS))
    {
      filter_status = "NOT_STARTED_INSUFFICIENT_TIME";
    }
    else
    {
      const double started = process_elapsed();
      for (Rung & rung : pass1)
      {
        filter_rung(&rung, threads);
      }
      for (Rung & rung : pass2)
      {
        filter_rung(&rung, threads);
      }
      filter_elapsed = process_elapsed() - started;
      filter_status = g_shared->time_exceeded.load() ? "GRAPH_INCOMPLETE" : "PASS";
    }
  }
  LadderOutcome pass1_result;
  if (filter_status == "PASS")
  {
    if (!begin_phase(kLadderS))
    {
      pass1_result.status = "NOT_STARTED_INSUFFICIENT_TIME";
    }
    else
    {
      pass1_result = run_ladder(pass1, false, threads);
    }
  }
  LadderOutcome pass2_result;
  if (pass1_result.status == "PATH")
  {
    if (!begin_phase(kLadderS))
    {
      pass2_result.status = "NOT_STARTED_INSUFFICIENT_TIME";
    }
    else
    {
      pass2_result = run_ladder(pass2, false, threads);
    }
  }
  std::string linear_status = "NOT_STARTED_PRIOR";
  double linear_elapsed = 0.0;
  Eigen::VectorXd q_69;
  Eigen::VectorXd q_70_independent;
  if (pass2_result.status == "PATH")
  {
    q_69 = pass1.back().candidates[pass1_result.chosen.back()].joints;
    q_70_independent = pass2.front().candidates[pass2_result.chosen.front()].joints;
    if (!begin_phase(kLinearS))
    {
      linear_status = "NOT_STARTED_INSUFFICIENT_TIME";
    }
    else
    {
      const double started = process_elapsed();
      zero_edge_counts();
      auto distance = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<double>>();
      LinearJointEdgeEvaluator evaluator(0.01 * max_joint_span(), distance);
      descartes_light::State<double> start_state(q_69);
      descartes_light::State<double> end_state(q_70_independent);
      const std::pair<bool, double> edge = evaluator.evaluate(start_state, end_state);
      linear_elapsed = process_elapsed() - started;
      if (g_shared->time_exceeded.load())
      {
        linear_status = "GRAPH_INCOMPLETE";
      }
      else if (g_shared->edge.self_error.load() > 0 || g_shared->edge.coupon_error.load() > 0)
      {
        linear_status = "EXECUTION_ERROR";
      }
      else
      {
        linear_status = edge.first ? "PASS" : "LINEAR_JOINT_EDGE_FAIL";
      }
    }
  }
  SampleJudge independent_process;
  SampleJudge independent_linear;
  std::string independent_post_status = "NOT_STARTED_PRIOR";
  if (pass2_result.status == "PATH" && linear_status != "NOT_STARTED_INSUFFICIENT_TIME" &&
      linear_status != "NOT_STARTED_PRIOR")
  {
    if (!begin_phase(kPostS))
    {
      independent_post_status = "NOT_STARTED_INSUFFICIENT_TIME";
    }
    else
    {
      const double started = process_elapsed();
      independent_process = judge_process_path(pass2, pass2_result.chosen);
      independent_linear = judge_linear_path(q_69, q_70_independent);
      independent_process.elapsed_s = process_elapsed() - started;
      independent_linear.elapsed_s = independent_process.elapsed_s;
      independent_post_status = g_shared->time_exceeded.load() ? "GRAPH_INCOMPLETE" : "DONE";
    }
  }
  std::vector<Rung> joint_rungs;
  LadderOutcome joint_result;
  if (pass2_result.status == "PATH")
  {
    if (!begin_phase(kLadderS))
    {
      joint_result.status = "NOT_STARTED_INSUFFICIENT_TIME";
    }
    else
    {
      Rung anchor;
      anchor.sequence_id = 69;
      anchor.position = pass1.back().position;
      anchor.orientation = pass1.back().orientation;
      Candidate only;
      only.joints = q_69;
      anchor.candidates.push_back(only);
      anchor.survivors.push_back(0);
      joint_rungs.push_back(anchor);
      joint_rungs.insert(joint_rungs.end(), pass2.begin(), pass2.end());
      joint_result = run_ladder(joint_rungs, true, threads);
    }
  }
  SampleJudge joint_process;
  SampleJudge joint_linear;
  std::string joint_post_status = "NOT_STARTED_PRIOR";
  std::vector<std::size_t> joint_process_chosen;
  if (joint_result.status == "PATH")
  {
    if (!begin_phase(kPostS))
    {
      joint_post_status = "NOT_STARTED_INSUFFICIENT_TIME";
    }
    else
    {
      const double started = process_elapsed();
      std::vector<Rung> process_only(joint_rungs.begin() + 1, joint_rungs.end());
      joint_process_chosen.assign(joint_result.chosen.begin() + 1, joint_result.chosen.end());
      joint_process = judge_process_path(process_only, joint_process_chosen);
      const Eigen::VectorXd q_70_joint = joint_rungs[1].candidates[joint_result.chosen[1]].joints;
      joint_linear = judge_linear_path(q_69, q_70_joint);
      joint_process.elapsed_s = process_elapsed() - started;
      joint_linear.elapsed_s = joint_process.elapsed_s;
      joint_post_status = g_shared->time_exceeded.load() ? "GRAPH_INCOMPLETE" : "DONE";
    }
  }
  const bool independent_valid = independent_post_status == "DONE" &&
    independent_process.status == "POSTCHECK_PASS" && independent_linear.status == "POSTCHECK_PASS" &&
    linear_status == "PASS";
  const bool joint_valid = joint_post_status == "DONE" && joint_process.status == "POSTCHECK_PASS" &&
    joint_linear.status == "POSTCHECK_PASS";
  std::string conclusion = "NOT_ANSWERED";
  if (pass1_result.status == "NO_PATH_IN_COMPLETED_GRAPH")
  {
    conclusion = "비교 불가. 이전 pass 공정 관절열이 완성 그래프에 없음";
  }
  else if (pass2_result.status == "NO_PATH_IN_COMPLETED_GRAPH")
  {
    conclusion = "저장된 후보 안에서 70을 포함한 pass 2 공정 관절열을 못 찾음";
  }
  else if (independent_valid && joint_valid)
  {
    conclusion = "고정 q_69와 이번 후보·직선 연결 범위에서 두 방식 모두 유효 조합";
  }
  else if (joint_valid)
  {
    conclusion = "고정 q_69와 이번 후보·직선 연결 범위에서 공동 선택의 유효 조합 확보";
  }
  else if (independent_valid)
  {
    conclusion = "고정 q_69와 이번 후보·직선 연결 범위에서 독립 선택만 유효 조합";
  }
  else if (pass2_result.status == "PATH" && independent_post_status == "DONE" && joint_post_status == "DONE")
  {
    conclusion = "두 방식 모두 이번 범위에서 유효 조합이 없음. 일반 자유 이동과 다른 q_69는 부정하지 않음";
  }
  else if (pass1_result.status == "GRAPH_INCOMPLETE" || pass2_result.status == "GRAPH_INCOMPLETE" ||
           joint_result.status == "GRAPH_INCOMPLETE" || filter_status == "GRAPH_INCOMPLETE" ||
           linear_status == "GRAPH_INCOMPLETE" || independent_post_status == "GRAPH_INCOMPLETE" ||
           joint_post_status == "GRAPH_INCOMPLETE")
  {
    conclusion = "이번 예산에서 미확보";
  }
  std::string save_status = "NOT_STARTED_PRIOR";
  const bool save_started = begin_phase(kSaveS);
  save_status = save_started ? "STARTED" : "NOT_STARTED_INSUFFICIENT_TIME";
  std::ofstream output(g_out_path);
  output << std::setprecision(17);
  output << "{\n  \"gate\": \"UR-ANALYTIC-IK-PROCESS-BOUNDARY-Q69\",\n"
         << "  \"question\": \"LINEAR_JOINT_EDGE_WITH_SAME_Q69\",\n"
         << "  \"joint_selection\": \"BOUNDARY_CANDIDATE_AND_PASS2_ONLY\",\n"
         << "  \"setup\": \"" << setup_status << "\",\n"
         << "  \"threads\": " << threads << ",\n"
         << "  \"max_joint_span_rad\": ";
  write_number(output, setup_status == "PASS" ? max_joint_span() : std::numeric_limits<double>::quiet_NaN());
  output << ",\n  \"budgets_s\": {\"overall\": 550, \"prep\": 10, \"filter\": 20, \"pass1_ladder\": 150,"
         << " \"pass2_ladder\": 150, \"linear_edge\": 10, \"independent_postcheck\": 20, \"joint_ladder\": 150,"
         << " \"joint_postcheck\": 20, \"save\": 20},\n"
         << "  \"leftover_transfer\": false,\n"
         << "  \"filter_status\": \"" << filter_status << "\",\n"
         << "  \"filter_elapsed_s\": ";
  write_number(output, filter_elapsed);
  output << ",\n  \"pass1\": {\"status\": \"" << pass1_result.status << "\", \"elapsed_s\": ";
  write_number(output, pass1_result.elapsed_s);
  output << ", \"edge_ceiling\": " << pass1_result.ceiling
         << ", \"edge_evaluations\": " << pass1_result.evaluations
         << ", \"ladder_cost\": ";
  write_number(output, pass1_result.cost);
  output << "},\n  \"pass2\": {\"status\": \"" << pass2_result.status << "\", \"elapsed_s\": ";
  write_number(output, pass2_result.elapsed_s);
  output << ", \"edge_ceiling\": " << pass2_result.ceiling
         << ", \"edge_evaluations\": " << pass2_result.evaluations
         << ", \"ladder_cost\": ";
  write_number(output, pass2_result.cost);
  output << "},\n  \"independent_linear_status\": \"" << linear_status << "\",\n"
         << "  \"independent_linear_elapsed_s\": ";
  write_number(output, linear_elapsed);
  output << ",\n  \"independent_post_status\": \"" << independent_post_status << "\",\n"
         << "  \"independent_process_postcheck\": ";
  write_judge(output, independent_process);
  output << ",\n  \"independent_linear_postcheck\": ";
  write_judge(output, independent_linear);
  output << ",\n  \"independent_valid\": " << (independent_valid ? "true" : "false")
         << ",\n  \"joint\": {\"status\": \"" << joint_result.status << "\", \"elapsed_s\": ";
  write_number(output, joint_result.elapsed_s);
  output << ", \"edge_ceiling\": " << joint_result.ceiling
         << ", \"edge_evaluations\": " << joint_result.evaluations
         << ", \"ladder_cost\": ";
  write_number(output, joint_result.cost);
  output << "},\n  \"joint_post_status\": \"" << joint_post_status << "\",\n"
         << "  \"joint_process_postcheck\": ";
  write_judge(output, joint_process);
  output << ",\n  \"joint_linear_postcheck\": ";
  write_judge(output, joint_linear);
  output << ",\n  \"joint_valid\": " << (joint_valid ? "true" : "false")
         << ",\n  \"conclusion\": \"" << conclusion << "\",\n"
         << "  \"free_motion_denied\": false,\n"
         << "  \"other_q69_denied\": false,\n"
         << "  \"linear_fail_is_transition_fail\": false,\n"
         << "  \"integrated_manifest\": \"UNCHANGED\",\n"
         << "  \"passing_record_count\": 6,\n"
         << "  \"p022\": \"UNCHANGED\",\n"
         << "  \"save_status\": \"" << save_status << "\",\n"
         << "  \"process_elapsed_s\": ";
  write_number(output, process_elapsed());
  output << ",\n  \"independent_selected\": ";
  if (pass2_result.status == "PATH")
  {
    write_choice(output, pass2, pass2_result.chosen);
  }
  else
  {
    output << "[]";
  }
  output << ",\n  \"joint_selected\": ";
  if (joint_result.status == "PATH")
  {
    write_choice(output, joint_rungs, joint_result.chosen);
  }
  else
  {
    output << "[]";
  }
  output << ",\n  \"q_69\": [";
  if (q_69.size() == 6)
  {
    for (int joint = 0; joint < 6; ++joint)
    {
      if (joint)
      {
        output << ", ";
      }
      write_number(output, q_69[joint]);
    }
  }
  output << "],\n  \"complete\": true\n}\n";
  if (setup_status != "PASS")
  {
    return 4;
  }
  if (conclusion == "이번 예산에서 미확보")
  {
    return 3;
  }
  return 0;
}

struct MemSnap
{
  long vm_hwm_kb = -1;
  long vm_rss_kb = -1;
};

struct EdgeKind
{
  bool transition = false;
  int original_from = -1;
  int original_to = -1;
  int half = 0;
};

MemSnap read_mem()
{
  MemSnap snap;
  std::ifstream input("/proc/self/status");
  std::string line;
  while (std::getline(input, line))
  {
    if (line.compare(0, 6, "VmHWM:") == 0)
    {
      snap.vm_hwm_kb = std::strtol(line.c_str() + 6, nullptr, 10);
    }
    else if (line.compare(0, 6, "VmRSS:") == 0)
    {
      snap.vm_rss_kb = std::strtol(line.c_str() + 6, nullptr, 10);
    }
  }
  return snap;
}

bool load_transitions_all(const std::string & path, std::map<int, Waypoint> * by_id)
{
  const int expected_ids[20] = {
    35, 70, 105, 140, 175, 210, 245, 281, 317, 353, 389, 425, 461, 496, 531, 566, 601, 636, 671, 706};
  std::ifstream input(path, std::ios::binary);
  char magic[8] = {};
  input.read(magic, 8);
  std::uint32_t count = 0;
  if (!input || std::string(magic, 4) != "TRN1" || !read_pod(input, &count) || count != 20)
  {
    return false;
  }
  for (int index = 0; index < 20; ++index)
  {
    std::int32_t stored_index = 0;
    std::int32_t from_run = 0;
    std::int32_t to_run = 0;
    std::int32_t sequence_id = 0;
    if (!read_pod(input, &stored_index) || !read_pod(input, &from_run) || !read_pod(input, &to_run) ||
        !read_pod(input, &sequence_id) || stored_index != index || sequence_id != expected_ids[index])
    {
      return false;
    }
    double ignored_from[6];
    double ignored_to[6];
    double position[3];
    double quaternion[4];
    input.read(reinterpret_cast<char *>(ignored_from), sizeof(ignored_from));
    input.read(reinterpret_cast<char *>(ignored_to), sizeof(ignored_to));
    input.read(reinterpret_cast<char *>(position), sizeof(position));
    input.read(reinterpret_cast<char *>(quaternion), sizeof(quaternion));
    std::uint32_t candidate_count = 0;
    if (!read_pod(input, &candidate_count) || candidate_count != 256)
    {
      return false;
    }
    Waypoint waypoint;
    waypoint.sequence_id = sequence_id;
    waypoint.position = Eigen::Vector3d(position[0], position[1], position[2]);
    waypoint.orientation = Eigen::Quaterniond(quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
    if (!waypoint.position.allFinite() || !(waypoint.orientation.norm() > 0.0))
    {
      return false;
    }
    waypoint.orientation.normalize();
    waypoint.candidates.resize(256);
    for (int candidate_index = 0; candidate_index < 256; ++candidate_index)
    {
      std::uint8_t source = 0;
      std::int32_t class_id = 0;
      if (!read_pod(input, &source) || !read_pod(input, &class_id) || (source != 0 && source != 1))
      {
        return false;
      }
      double joints[6];
      input.read(reinterpret_cast<char *>(joints), sizeof(joints));
      Candidate & candidate = waypoint.candidates[static_cast<std::size_t>(candidate_index)];
      candidate.source_code = source;
      candidate.kinematic_class_id = class_id;
      candidate.joints = Eigen::VectorXd(6);
      for (int joint = 0; joint < 6; ++joint)
      {
        if (!std::isfinite(joints[joint]))
        {
          return false;
        }
        candidate.joints[joint] = joints[joint];
      }
    }
    (*by_id)[sequence_id] = std::move(waypoint);
  }
  return static_cast<bool>(input) && input.peek() == EOF && by_id->size() == 20;
}

bool expected_insert_span(int from_id, int to_id)
{
  const int spans[6][3] = {{6, 211, 244}, {7, 246, 280}, {8, 282, 316}, {9, 318, 352}, {10, 354, 388}, {11, 390, 424}};
  if (to_id != from_id + 1)
  {
    return false;
  }
  for (const auto & span : spans)
  {
    if (from_id >= span[1] && to_id <= span[2])
    {
      return true;
    }
  }
  return false;
}

bool load_r6_inserts(const std::string & path, std::map<int, Insert> * inserts)
{
  const int run_ids[6] = {6, 7, 8, 9, 10, 11};
  const int counts[6] = {33, 34, 34, 34, 34, 34};
  std::ifstream input(path, std::ios::binary);
  char magic[4] = {};
  input.read(magic, 4);
  std::uint32_t run_count = 0;
  if (!input || std::string(magic, 4) != "R6IN" || !read_pod(input, &run_count) || run_count != 6)
  {
    return false;
  }
  for (int index = 0; index < 6; ++index)
  {
    std::int32_t run_id = 0;
    std::int32_t insert_count = 0;
    if (!read_pod(input, &run_id) || !read_pod(input, &insert_count) || run_id != run_ids[index] ||
        insert_count != counts[index])
    {
      return false;
    }
    for (int insert_index = 0; insert_index < insert_count; ++insert_index)
    {
      std::int32_t from_id = 0;
      std::int32_t to_id = 0;
      if (!read_pod(input, &from_id) || !read_pod(input, &to_id) || !expected_insert_span(from_id, to_id))
      {
        return false;
      }
      double position[3];
      double orientation[4];
      input.read(reinterpret_cast<char *>(position), sizeof(position));
      input.read(reinterpret_cast<char *>(orientation), sizeof(orientation));
      std::uint32_t valid_count = 0;
      if (!read_pod(input, &valid_count) || valid_count != 256)
      {
        return false;
      }
      Insert insert;
      insert.from_sequence_id = from_id;
      insert.to_sequence_id = to_id;
      insert.position = Eigen::Vector3d(position[0], position[1], position[2]);
      insert.orientation = Eigen::Quaterniond(orientation[3], orientation[0], orientation[1], orientation[2]);
      if (!insert.position.allFinite() || !(insert.orientation.norm() > 0.0))
      {
        return false;
      }
      insert.orientation.normalize();
      insert.valid.resize(256);
      for (Candidate & candidate : insert.valid)
      {
        std::int32_t class_id = 0;
        std::uint8_t source = 0;
        if (!read_pod(input, &class_id) || !read_pod(input, &source) || (source != 0 && source != 1))
        {
          return false;
        }
        candidate.kinematic_class_id = class_id;
        candidate.source_code = source;
        double joints[6];
        input.read(reinterpret_cast<char *>(joints), sizeof(joints));
        candidate.joints = Eigen::VectorXd(6);
        for (int joint = 0; joint < 6; ++joint)
        {
          if (!std::isfinite(joints[joint]))
          {
            return false;
          }
          candidate.joints[joint] = joints[joint];
        }
      }
      if (!inserts->emplace(from_id, std::move(insert)).second)
      {
        return false;
      }
    }
  }
  return static_cast<bool>(input) && input.peek() == EOF && inserts->size() == 203;
}

bool assemble_full(
  const std::vector<std::vector<Waypoint>> & runs,
  const std::string & transitions_path,
  const std::string & inserts_path,
  std::vector<Rung> * rungs,
  std::vector<EdgeKind> * edges)
{
  const int transition_ids[20] = {
    35, 70, 105, 140, 175, 210, 245, 281, 317, 353, 389, 425, 461, 496, 531, 566, 601, 636, 671, 706};
  std::map<int, Waypoint> by_id;
  if (!load_transitions_all(transitions_path, &by_id))
  {
    return false;
  }
  for (const auto & run : runs)
  {
    for (const Waypoint & waypoint : run)
    {
      if (!by_id.emplace(waypoint.sequence_id, waypoint).second)
      {
        return false;
      }
    }
  }
  if (by_id.size() != 741)
  {
    return false;
  }
  for (int sequence = 0; sequence < 741; ++sequence)
  {
    if (by_id.count(sequence) != 1)
    {
      return false;
    }
  }
  std::map<int, Insert> inserts;
  if (!load_r6_inserts(inserts_path, &inserts))
  {
    return false;
  }
  auto push_original = [&](int sequence) {
    const Waypoint & waypoint = by_id.at(sequence);
    Rung rung;
    rung.sequence_id = sequence;
    rung.position = waypoint.position;
    rung.orientation = waypoint.orientation;
    rung.candidates = waypoint.candidates;
    rungs->push_back(std::move(rung));
  };
  push_original(0);
  int process_edges = 0;
  int transition_edges = 0;
  int split_edges = 0;
  for (int sequence = 1; sequence < 741; ++sequence)
  {
    const bool transition = std::find(std::begin(transition_ids), std::end(transition_ids), sequence) !=
      std::end(transition_ids);
    const auto insert = inserts.find(sequence - 1);
    if (insert != inserts.end())
    {
      if (transition || insert->second.to_sequence_id != sequence)
      {
        return false;
      }
      Rung mid;
      mid.inserted = true;
      mid.from_sequence_id = sequence - 1;
      mid.to_sequence_id = sequence;
      mid.position = insert->second.position;
      mid.orientation = insert->second.orientation;
      mid.candidates = insert->second.valid;
      rungs->push_back(std::move(mid));
      edges->push_back(EdgeKind{false, sequence - 1, sequence, 1});
      push_original(sequence);
      edges->push_back(EdgeKind{false, sequence - 1, sequence, 2});
      process_edges += 2;
      split_edges += 2;
    }
    else
    {
      push_original(sequence);
      edges->push_back(EdgeKind{transition, sequence - 1, sequence, 0});
      if (transition)
      {
        ++transition_edges;
      }
      else
      {
        ++process_edges;
      }
    }
  }
  return rungs->size() == 944 && edges->size() == 943 && process_edges == 923 && transition_edges == 20 &&
    split_edges == 406 && inserts.size() == 203;
}

void judge_process_edge(const Rung & start, const Rung & end, std::size_t start_index, std::size_t end_index, SampleJudge * judge)
{
  const Eigen::VectorXd & start_joints = start.candidates[start_index].joints;
  const Eigen::VectorXd & end_joints = end.candidates[end_index].joints;
  for (int sample_index = 0; sample_index <= kSampleDenominator; ++sample_index)
  {
    if (timed_out())
    {
      judge->status = "GRAPH_INCOMPLETE";
      judge->error = true;
      return;
    }
    const double parameter = static_cast<double>(sample_index) / static_cast<double>(kSampleDenominator);
    const Eigen::VectorXd joints = (1.0 - parameter) * start_joints + parameter * end_joints;
    ++judge->samples;
    if (sample_index == 0 || sample_index == kSampleDenominator)
    {
      const Eigen::VectorXd & endpoint = sample_index == 0 ? start_joints : end_joints;
      judge->junction_rad = std::max(judge->junction_rad, raw_difference(joints, endpoint));
      if (!(judge->junction_rad <= 1.0e-12))
      {
        judge->junction = false;
      }
    }
    if (!joints.allFinite() || !within_limits(joints))
    {
      judge->limits = false;
      ++judge->limit_fail;
      ++judge->collision_not_evaluated;
      continue;
    }
    double position_error = 0.0;
    double orientation_error = 0.0;
    const Eigen::Vector3d target = (1.0 - parameter) * start.position + parameter * end.position;
    const Eigen::Quaterniond orientation = start.orientation.slerp(parameter, end.orientation);
    const bool accurate = pose_error(joints, target, orientation, &position_error, &orientation_error) &&
      position_error <= kPathPositionM && orientation_error <= kPathOrientationRad;
    note_sample_error(judge, position_error, orientation_error);
    if (!accurate)
    {
      judge->accuracy = false;
      ++judge->accuracy_fail;
      ++judge->collision_not_evaluated;
      continue;
    }
    bool self_hit = false;
    bool coupon_hit = false;
    bool error = false;
    apply_collision(&g_shared->edge, joints, &self_hit, &coupon_hit, &error);
    if (error)
    {
      judge->error = true;
      ++judge->collision_not_evaluated;
      continue;
    }
    if (self_hit)
    {
      judge->self_clear = false;
      ++judge->self_hit;
      ++judge->collision_not_evaluated;
      continue;
    }
    if (coupon_hit)
    {
      judge->coupon_clear = false;
      ++judge->coupon_hit;
    }
  }
}

int run_full(const std::vector<std::vector<Waypoint>> & runs, const std::string & transitions_path, const std::string & inserts_path, int threads)
{
  std::string setup_status = threads == 24 ? "PASS" : "NOT_STARTED_THREAD_COUNT";
  std::vector<Rung> rungs;
  std::vector<EdgeKind> edges;
  if (setup_status == "PASS" && (!verify_threads(threads) || !assemble_full(runs, transitions_path, inserts_path, &rungs, &edges)))
  {
    setup_status = "BLOCKED_INPUT_INTEGRITY";
  }
  if (setup_status == "PASS" && process_elapsed() > kFullPrepS)
  {
    setup_status = "NOT_STARTED_INSUFFICIENT_TIME";
  }
  std::string filter_status = "NOT_STARTED_PRIOR";
  double filter_elapsed = 0.0;
  MemSnap filter_mem;
  std::uint64_t survivor_sum = 0;
  std::uint64_t survivor_min = 0;
  std::uint64_t survivor_max = 0;
  if (setup_status == "PASS")
  {
    if (!begin_phase(kFullFilterS))
    {
      filter_status = "NOT_STARTED_INSUFFICIENT_TIME";
    }
    else
    {
      const double started = process_elapsed();
      for (Rung & rung : rungs)
      {
        filter_rung(&rung, threads);
      }
      filter_elapsed = process_elapsed() - started;
      filter_mem = read_mem();
      filter_status = g_shared->time_exceeded.load() || g_shared->rung.evaluations.load() != 944ULL * 256ULL ?
        "GRAPH_INCOMPLETE" : "PASS";
      if (filter_status == "PASS")
      {
        survivor_min = rungs.front().survivors.size();
        for (const Rung & rung : rungs)
        {
          const std::uint64_t count = rung.survivors.size();
          survivor_sum += count;
          survivor_min = std::min(survivor_min, count);
          survivor_max = std::max(survivor_max, count);
        }
      }
    }
  }
  LadderOutcome ladder;
  MemSnap ladder_mem;
  if (filter_status == "PASS")
  {
    if (!begin_phase(kFullLadderS))
    {
      ladder.status = "NOT_STARTED_INSUFFICIENT_TIME";
    }
    else
    {
      const double started = process_elapsed();
      g_accepted_edges.store(0);
      bool missing = false;
      for (std::size_t index = 0; index + 1 < rungs.size(); ++index)
      {
        const std::uint64_t left = rungs[index].survivors.size();
        const std::uint64_t right = rungs[index + 1].survivors.size();
        if (left == 0 || right == 0)
        {
          missing = true;
        }
        else if (right > std::numeric_limits<std::uint64_t>::max() / left ||
                 ladder.ceiling > std::numeric_limits<std::uint64_t>::max() - left * right)
        {
          ladder.ceiling_overflow = true;
        }
        else
        {
          ladder.ceiling += left * right;
        }
      }
      if (ladder.ceiling_overflow)
      {
        ladder.status = "EXECUTION_ERROR";
      }
      else if (missing)
      {
        ladder.status = "NO_PATH_IN_COMPLETED_GRAPH";
      }
      else
      {
        zero_edge_counts();
        std::vector<std::vector<descartes_light::State<double>::ConstPtr>> states(rungs.size());
        std::vector<descartes_light::WaypointSampler<double>::ConstPtr> samplers;
        std::vector<descartes_light::EdgeEvaluator<double>::ConstPtr> gates;
        auto distance = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<double>>();
        for (std::size_t rung_index = 0; rung_index < rungs.size(); ++rung_index)
        {
          for (std::size_t survivor : rungs[rung_index].survivors)
          {
            states[rung_index].push_back(std::make_shared<descartes_light::State<double>>(
              rungs[rung_index].candidates[survivor].joints));
          }
          samplers.push_back(std::make_shared<RecordedJointSampler>(states[rung_index]));
        }
        const double step = 0.01 * max_joint_span();
        for (std::size_t segment = 0; segment < edges.size(); ++segment)
        {
          if (edges[segment].transition)
          {
            gates.push_back(std::make_shared<LinearJointEdgeEvaluator>(step, distance));
          }
          else
          {
            gates.push_back(std::make_shared<CollisionEdgeEvaluator>(
              rungs[segment].position, rungs[segment + 1].position,
              rungs[segment].orientation, rungs[segment + 1].orientation,
              0.0, 1.0, distance));
          }
        }
        try
        {
          descartes_light::LadderGraphSolver<double> solver(threads);
          const auto built = solver.build(samplers, gates, {});
          ladder.evaluations = g_shared->edge.evaluations.load();
          if (g_shared->time_exceeded.load())
          {
            ladder.status = "GRAPH_INCOMPLETE";
          }
          else if (g_shared->setup_failed.load() || g_shared->edge.self_error.load() > 0 ||
                   g_shared->edge.coupon_error.load() > 0)
          {
            ladder.status = "EXECUTION_ERROR";
          }
          else if (ladder.evaluations != ladder.ceiling)
          {
            ladder.status = "GRAPH_INCOMPLETE";
          }
          else if (!built)
          {
            ladder.status = "NO_PATH_IN_COMPLETED_GRAPH";
          }
          else
          {
            const descartes_light::SearchResult<double> result = solver.search();
            if (g_shared->time_exceeded.load())
            {
              ladder.status = "GRAPH_INCOMPLETE";
            }
            else if (result.trajectory.size() != rungs.size() || !std::isfinite(result.cost))
            {
              ladder.status = "EXECUTION_ERROR";
            }
            else
            {
              ladder.chosen.resize(rungs.size());
              bool matched = true;
              for (std::size_t rung_index = 0; rung_index < rungs.size(); ++rung_index)
              {
                const auto found = std::find(
                  states[rung_index].begin(), states[rung_index].end(), result.trajectory[rung_index]);
                if (found == states[rung_index].end())
                {
                  matched = false;
                  break;
                }
                ladder.chosen[rung_index] = rungs[rung_index].survivors[static_cast<std::size_t>(
                  found - states[rung_index].begin())];
              }
              ladder.status = matched ? "PATH" : "EXECUTION_ERROR";
              if (matched)
              {
                ladder.cost = result.cost;
              }
            }
          }
        }
        catch (const std::bad_alloc &)
        {
          ladder.status = "ALLOCATION_FAILURE";
        }
        catch (const std::runtime_error &)
        {
          ladder.evaluations = g_shared->edge.evaluations.load();
          ladder.status = ladder.evaluations == ladder.ceiling && !g_shared->time_exceeded.load() ?
            "NO_PATH_IN_COMPLETED_GRAPH" : "GRAPH_INCOMPLETE";
        }
        catch (const std::exception &)
        {
          ladder.status = "EXECUTION_ERROR";
        }
      }
      if (ladder.status != "PATH")
      {
        ladder.chosen.clear();
      }
      ladder.elapsed_s = process_elapsed() - started;
      ladder_mem = read_mem();
    }
  }
  SampleJudge process_judge;
  SampleJudge transition_judge;
  transition_judge.accuracy_applied = false;
  std::string post_status = "NOT_STARTED_PRIOR";
  double post_elapsed = 0.0;
  MemSnap post_mem;
  int original_process = 0;
  int original_transition = 0;
  if (ladder.status == "PATH")
  {
    if (!begin_phase(kFullPostS))
    {
      post_status = "NOT_STARTED_INSUFFICIENT_TIME";
    }
    else
    {
      const double started = process_elapsed();
      std::map<std::pair<int, int>, int> process_halves;
      for (std::size_t segment = 0; segment < edges.size(); ++segment)
      {
        if (process_judge.status == "GRAPH_INCOMPLETE" || transition_judge.status == "GRAPH_INCOMPLETE")
        {
          break;
        }
        const EdgeKind & edge = edges[segment];
        if (edge.transition)
        {
          SampleJudge one = judge_linear_path(
            rungs[segment].candidates[ladder.chosen[segment]].joints,
            rungs[segment + 1].candidates[ladder.chosen[segment + 1]].joints);
          if (one.status == "GRAPH_INCOMPLETE")
          {
            transition_judge.status = one.status;
            transition_judge.error = true;
            break;
          }
          transition_judge.samples += one.samples;
          transition_judge.limit_fail += one.limit_fail;
          transition_judge.self_hit += one.self_hit;
          transition_judge.coupon_hit += one.coupon_hit;
          transition_judge.collision_not_evaluated += one.collision_not_evaluated;
          transition_judge.limits = transition_judge.limits && one.limits;
          transition_judge.self_clear = transition_judge.self_clear && one.self_clear;
          transition_judge.coupon_clear = transition_judge.coupon_clear && one.coupon_clear;
          transition_judge.junction = transition_judge.junction && one.junction;
          transition_judge.error = transition_judge.error || one.error;
          transition_judge.junction_rad = std::max(transition_judge.junction_rad, one.junction_rad);
          ++original_transition;
        }
        else
        {
          judge_process_edge(
            rungs[segment], rungs[segment + 1], ladder.chosen[segment], ladder.chosen[segment + 1], &process_judge);
          process_halves[{edge.original_from, edge.original_to}] += 1;
        }
      }
      for (const auto & item : process_halves)
      {
        if (item.second == 1 || item.second == 2)
        {
          ++original_process;
        }
      }
      const bool covered = original_process == 720 && original_transition == 20 && process_halves.size() == 720;
      if (process_judge.status != "GRAPH_INCOMPLETE" && transition_judge.status != "GRAPH_INCOMPLETE")
      {
        process_judge.status = covered && process_judge.accuracy && process_judge.limits && process_judge.self_clear &&
          process_judge.coupon_clear && process_judge.junction && !process_judge.error &&
          process_judge.collision_not_evaluated == 0 ? "POSTCHECK_PASS" : "POSTCHECK_FAIL";
        transition_judge.status = transition_judge.limits && transition_judge.self_clear &&
          transition_judge.coupon_clear && transition_judge.junction && !transition_judge.error &&
          transition_judge.collision_not_evaluated == 0 ? "POSTCHECK_PASS" : "POSTCHECK_FAIL";
        if (!covered)
        {
          process_judge.status = "POSTCHECK_FAIL";
        }
      }
      post_elapsed = process_elapsed() - started;
      post_mem = read_mem();
      post_status = g_shared->time_exceeded.load() || process_judge.status == "GRAPH_INCOMPLETE" ||
        transition_judge.status == "GRAPH_INCOMPLETE" ? "GRAPH_INCOMPLETE" : "DONE";
    }
  }
  const bool discrete_pass = ladder.status == "PATH" && post_status == "DONE" &&
    process_judge.status == "POSTCHECK_PASS" && transition_judge.status == "POSTCHECK_PASS";
  std::string conclusion = "NOT_ANSWERED";
  if (discrete_pass)
  {
    conclusion = "저장 후보와 직선 TRANSITION을 사용한 전체 기하 후보 경로의 이산 검사 통과";
  }
  else if (ladder.status == "NO_PATH_IN_COMPLETED_GRAPH")
  {
    conclusion = "완성된 제한 그래프에 전체 경로가 없음. 일반 자유 이동은 부정하지 않음";
  }
  else if (ladder.status == "ALLOCATION_FAILURE" || ladder.status == "GRAPH_INCOMPLETE" ||
           filter_status == "GRAPH_INCOMPLETE" || post_status == "GRAPH_INCOMPLETE")
  {
    conclusion = "이번 예산에서 미확보";
  }
  else if (post_status == "DONE")
  {
    conclusion = "선택 경로의 독립 검사가 통과하지 않음. 일반 자유 이동은 부정하지 않음";
  }
  const bool save_started = begin_phase(kFullSaveS);
  const MemSnap save_mem = read_mem();
  struct rusage usage;
  getrusage(RUSAGE_SELF, &usage);
  std::ofstream output(g_out_path);
  output << std::setprecision(17);
  output << "{\n  \"gate\": \"UR-ANALYTIC-IK-PROCESS-FULL-LAYERED-GRAPH\",\n"
         << "  \"setup\": \"" << setup_status << "\",\n"
         << "  \"threads\": " << threads << ",\n"
         << "  \"rungs\": " << rungs.size() << ",\n"
         << "  \"graph_edges\": " << edges.size() << ",\n"
         << "  \"inserts\": 203,\n"
         << "  \"original_process_edges\": 720,\n"
         << "  \"original_transition_edges\": 20,\n"
         << "  \"filter_status\": \"" << filter_status << "\",\n"
         << "  \"filter_elapsed_s\": ";
  write_number(output, filter_elapsed);
  output << ",\n  \"filter_evaluations\": " << g_shared->rung.evaluations.load()
         << ",\n  \"survivor_sum\": " << survivor_sum
         << ",\n  \"survivor_min\": " << survivor_min
         << ",\n  \"survivor_max\": " << survivor_max
         << ",\n  \"filter_vm_rss_kb\": " << filter_mem.vm_rss_kb
         << ",\n  \"filter_vm_hwm_kb\": " << filter_mem.vm_hwm_kb
         << ",\n  \"ladder_status\": \"" << ladder.status << "\",\n"
         << "  \"ladder_elapsed_s\": ";
  write_number(output, ladder.elapsed_s);
  output << ",\n  \"edge_ceiling\": " << ladder.ceiling
         << ",\n  \"edge_evaluations\": " << ladder.evaluations
         << ",\n  \"stored_edges\": " << g_accepted_edges.load()
         << ",\n  \"ladder_cost\": ";
  write_number(output, ladder.cost);
  output << ",\n  \"ladder_vm_rss_kb\": " << ladder_mem.vm_rss_kb
         << ",\n  \"ladder_vm_hwm_kb\": " << ladder_mem.vm_hwm_kb
         << ",\n  \"post_status\": \"" << post_status << "\",\n"
         << "  \"post_elapsed_s\": ";
  write_number(output, post_elapsed);
  output << ",\n  \"post_original_process\": " << original_process
         << ",\n  \"post_original_transition\": " << original_transition
         << ",\n  \"process_postcheck\": ";
  write_judge(output, process_judge);
  output << ",\n  \"transition_postcheck\": ";
  write_judge(output, transition_judge);
  output << ",\n  \"post_vm_rss_kb\": " << post_mem.vm_rss_kb
         << ",\n  \"post_vm_hwm_kb\": " << post_mem.vm_hwm_kb
         << ",\n  \"max_rss_kb\": " << usage.ru_maxrss
         << ",\n  \"save_vm_rss_kb\": " << save_mem.vm_rss_kb
         << ",\n  \"save_vm_hwm_kb\": " << save_mem.vm_hwm_kb
         << ",\n  \"discrete_pass\": " << (discrete_pass ? "true" : "false")
         << ",\n  \"conclusion\": \"" << conclusion << "\",\n"
         << "  \"continuous_safety\": \"NOT_CLAIMED\",\n"
         << "  \"time_parameterization\": \"NOT_CLAIMED\",\n"
         << "  \"physical_execution\": \"NOT_CLAIMED\",\n"
         << "  \"free_motion_denied\": false,\n"
         << "  \"integrated_manifest\": \"UNCHANGED\",\n"
         << "  \"passing_record_count\": 6,\n"
         << "  \"p022\": \"UNCHANGED\",\n"
         << "  \"save_status\": \"" << (save_started ? "STARTED" : "NOT_STARTED_INSUFFICIENT_TIME") << "\",\n"
         << "  \"process_elapsed_s\": ";
  write_number(output, process_elapsed());
  output << ",\n  \"survivor_counts\": [";
  for (std::size_t index = 0; index < rungs.size(); ++index)
  {
    if (index)
    {
      output << ", ";
    }
    output << rungs[index].survivors.size();
  }
  output << "],\n  \"selected\": [";
  for (std::size_t index = 0; index < ladder.chosen.size(); ++index)
  {
    if (index)
    {
      output << ", ";
    }
    const Candidate & candidate = rungs[index].candidates[ladder.chosen[index]];
    output << "{\"rung\": " << index
           << ", \"inserted\": " << (rungs[index].inserted ? "true" : "false")
           << ", \"sequence_id\": " << rungs[index].sequence_id
           << ", \"from_sequence_id\": " << rungs[index].from_sequence_id
           << ", \"to_sequence_id\": " << rungs[index].to_sequence_id
           << ", \"candidate_index\": " << ladder.chosen[index] << "}";
  }
  output << "],\n  \"complete\": true\n}\n";
  if (setup_status == "BLOCKED_INPUT_INTEGRITY")
  {
    return 2;
  }
  if (setup_status != "PASS" || filter_status == "GRAPH_INCOMPLETE" || ladder.status == "GRAPH_INCOMPLETE" ||
      ladder.status == "ALLOCATION_FAILURE" || post_status == "GRAPH_INCOMPLETE" ||
      filter_status == "NOT_STARTED_INSUFFICIENT_TIME" || ladder.status == "NOT_STARTED_INSUFFICIENT_TIME")
  {
    return 3;
  }
  if (ladder.status == "EXECUTION_ERROR" || setup_status == "NOT_STARTED_THREAD_COUNT")
  {
    return setup_status == "NOT_STARTED_THREAD_COUNT" ? 3 : 4;
  }
  return 0;
}

struct SelectedSlot
{
  int rung = -1;
  bool inserted = false;
  int sequence_id = -1;
  int from_sequence_id = -1;
  int to_sequence_id = -1;
  int candidate_index = -1;
};

struct ContactRecord
{
  std::string first;
  std::string second;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double depth = 0.0;
};

struct SampleFailure
{
  int sample = -1;
  double parameter = 0.0;
  std::string reason;
  Eigen::VectorXd joints;
  bool position_applied = false;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  std::size_t contact_count = 0;
  std::vector<ContactRecord> contacts;
};

struct HalfRecord
{
  int half = 0;
  bool transition = false;
  int samples = 0;
  int nonfinite = 0;
  int limit_fail = 0;
  int accuracy_fail = 0;
  int self_hit = 0;
  int coupon_hit = 0;
  int collision_not_evaluated = 0;
  int error = 0;
  double max_position = std::numeric_limits<double>::quiet_NaN();
  double max_orientation = std::numeric_limits<double>::quiet_NaN();
  double endpoint_identity_rad = 0.0;
  bool endpoint_checked = false;
  std::string status = "NOT_EVALUATED";
  std::vector<SampleFailure> failures;
};

std::string sha256_text(const std::string & text)
{
  SHA256_CTX context;
  SHA256_Init(&context);
  SHA256_Update(&context, text.data(), text.size());
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256_Final(digest, &context);
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned char byte : digest)
  {
    output << std::setw(2) << static_cast<int>(byte);
  }
  return output.str();
}

std::string json_escape(const std::string & text)
{
  std::string out;
  for (unsigned char ch : text)
  {
    if (ch == '\\' || ch == '"')
    {
      out.push_back('\\');
      out.push_back(static_cast<char>(ch));
    }
    else if (ch < 0x20)
    {
      out.push_back('?');
    }
    else
    {
      out.push_back(static_cast<char>(ch));
    }
  }
  return out;
}

bool parse_selected(const std::string & text, std::vector<SelectedSlot> * selected, std::string * canonical)
{
  const std::string key = "\"selected\":";
  const std::size_t found = text.find(key);
  if (found == std::string::npos)
  {
    return false;
  }
  std::size_t cursor = found + key.size();
  auto skip = [&]() {
    while (cursor < text.size() && std::isspace(static_cast<unsigned char>(text[cursor])))
    {
      ++cursor;
    }
  };
  auto expect = [&](char token) {
    skip();
    if (cursor >= text.size() || text[cursor] != token)
    {
      return false;
    }
    ++cursor;
    return true;
  };
  auto parse_int = [&](int * value) {
    skip();
    if (cursor >= text.size())
    {
      return false;
    }
    const std::size_t start = cursor;
    if (text[cursor] == '-')
    {
      ++cursor;
    }
    if (cursor >= text.size() || !std::isdigit(static_cast<unsigned char>(text[cursor])))
    {
      return false;
    }
    while (cursor < text.size() && std::isdigit(static_cast<unsigned char>(text[cursor])))
    {
      ++cursor;
    }
    try
    {
      *value = std::stoi(text.substr(start, cursor - start));
    }
    catch (const std::exception &)
    {
      return false;
    }
    return true;
  };
  auto parse_bool = [&](bool * value) {
    skip();
    if (text.compare(cursor, 4, "true") == 0)
    {
      *value = true;
      cursor += 4;
      return true;
    }
    if (text.compare(cursor, 5, "false") == 0)
    {
      *value = false;
      cursor += 5;
      return true;
    }
    return false;
  };
  if (!expect('['))
  {
    return false;
  }
  std::ostringstream compact;
  compact << '[';
  skip();
  bool first = true;
  while (cursor < text.size() && text[cursor] != ']')
  {
    if (!expect('{'))
    {
      return false;
    }
    SelectedSlot slot;
    int seen = 0;
    while (cursor < text.size() && text[cursor] != '}')
    {
      skip();
      if (text.compare(cursor, 6, "\"rung\"") == 0)
      {
        cursor += 6;
        if (!expect(':') || !parse_int(&slot.rung))
        {
          return false;
        }
      }
      else if (text.compare(cursor, 10, "\"inserted\"") == 0)
      {
        cursor += 10;
        if (!expect(':') || !parse_bool(&slot.inserted))
        {
          return false;
        }
      }
      else if (text.compare(cursor, 13, "\"sequence_id\"") == 0)
      {
        cursor += 13;
        if (!expect(':') || !parse_int(&slot.sequence_id))
        {
          return false;
        }
      }
      else if (text.compare(cursor, 18, "\"from_sequence_id\"") == 0)
      {
        cursor += 18;
        if (!expect(':') || !parse_int(&slot.from_sequence_id))
        {
          return false;
        }
      }
      else if (text.compare(cursor, 16, "\"to_sequence_id\"") == 0)
      {
        cursor += 16;
        if (!expect(':') || !parse_int(&slot.to_sequence_id))
        {
          return false;
        }
      }
      else if (text.compare(cursor, 17, "\"candidate_index\"") == 0)
      {
        cursor += 17;
        if (!expect(':') || !parse_int(&slot.candidate_index))
        {
          return false;
        }
      }
      else
      {
        return false;
      }
      ++seen;
      skip();
      if (cursor < text.size() && text[cursor] == ',')
      {
        ++cursor;
      }
    }
    if (!expect('}') || seen != 6)
    {
      return false;
    }
    if (!first)
    {
      compact << ',';
    }
    first = false;
    compact << "{\"rung\":" << slot.rung
            << ",\"inserted\":" << (slot.inserted ? "true" : "false")
            << ",\"sequence_id\":" << slot.sequence_id
            << ",\"from_sequence_id\":" << slot.from_sequence_id
            << ",\"to_sequence_id\":" << slot.to_sequence_id
            << ",\"candidate_index\":" << slot.candidate_index << '}';
    selected->push_back(slot);
    skip();
    if (cursor < text.size() && text[cursor] == ',')
    {
      ++cursor;
      skip();
    }
  }
  if (!expect(']'))
  {
    return false;
  }
  compact << ']';
  *canonical = compact.str();
  return true;
}

std::string slot_filter_reason(const Candidate & candidate, const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation)
{
  if (!candidate.joints.allFinite())
  {
    return "NONFINITE";
  }
  if (!within_limits(candidate.joints))
  {
    return "LIMIT";
  }
  double position_error = 0.0;
  double orientation_error = 0.0;
  if (!pose_error(candidate.joints, position, orientation, &position_error, &orientation_error) ||
      position_error > kModelPositionM || orientation_error > kModelOrientationRad)
  {
    return "POSE";
  }
  bool self_hit = false;
  bool coupon_hit = false;
  bool error = false;
  Counts scratch;
  apply_collision(&scratch, candidate.joints, &self_hit, &coupon_hit, &error);
  if (error)
  {
    return "ERROR";
  }
  if (self_hit)
  {
    return "SELF";
  }
  if (coupon_hit)
  {
    return "COUPON";
  }
  return "IN_SURVIVORS";
}

void collect_contacts(bool self_collision, moveit::core::RobotState & state, SampleFailure * failure)
{
  collision_detection::CollisionResult result;
  LocalScene & local = local_scene();
  if (!local.ready || g_shared->setup_failed.load())
  {
    failure->reason = "ERROR";
    return;
  }
  try
  {
    if (self_collision)
    {
      local.scene->getCollisionEnv()->checkSelfCollision(g_shared->request, result, state, g_shared->acm);
    }
    else
    {
      local.scene->getCollisionEnv()->checkRobotCollision(g_shared->request, result, state, g_shared->acm);
    }
  }
  catch (const std::exception &)
  {
    failure->reason = "ERROR";
    return;
  }
  failure->contact_count = result.contact_count;
  for (const auto & entry : result.contacts)
  {
    for (const auto & contact : entry.second)
    {
      ContactRecord record;
      record.first = contact.body_name_1;
      record.second = contact.body_name_2;
      record.x = contact.pos.x();
      record.y = contact.pos.y();
      record.z = contact.pos.z();
      record.depth = contact.depth;
      failure->contacts.push_back(record);
    }
  }
}

void inspect_half(
  const Rung & start,
  const Rung & end,
  std::size_t start_index,
  std::size_t end_index,
  bool transition,
  HalfRecord * record)
{
  const Eigen::VectorXd & start_joints = start.candidates[start_index].joints;
  const Eigen::VectorXd & end_joints = end.candidates[end_index].joints;
  record->transition = transition;
  for (int sample_index = 0; sample_index <= kSampleDenominator; ++sample_index)
  {
    if (timed_out())
    {
      record->status = "NOT_EVALUATED";
      return;
    }
    const double parameter = static_cast<double>(sample_index) / static_cast<double>(kSampleDenominator);
    const Eigen::VectorXd joints = (1.0 - parameter) * start_joints + parameter * end_joints;
    ++record->samples;
    if (sample_index == 0 || sample_index == kSampleDenominator)
    {
      const Eigen::VectorXd & endpoint = sample_index == 0 ? start_joints : end_joints;
      record->endpoint_identity_rad = std::max(record->endpoint_identity_rad, raw_difference(joints, endpoint));
      record->endpoint_checked = true;
    }
    SampleFailure failure;
    failure.sample = sample_index;
    failure.parameter = parameter;
    failure.joints = joints;
    if (!transition)
    {
      failure.position_applied = true;
      failure.position = (1.0 - parameter) * start.position + parameter * end.position;
    }
    if (!joints.allFinite())
    {
      ++record->nonfinite;
      ++record->collision_not_evaluated;
      failure.reason = "NONFINITE";
      record->failures.push_back(failure);
      continue;
    }
    if (!within_limits(joints))
    {
      ++record->limit_fail;
      ++record->collision_not_evaluated;
      failure.reason = "LIMIT";
      record->failures.push_back(failure);
      continue;
    }
    if (!transition)
    {
      double position_error = 0.0;
      double orientation_error = 0.0;
      const Eigen::Quaterniond orientation = start.orientation.slerp(parameter, end.orientation);
      const bool accurate = pose_error(joints, failure.position, orientation, &position_error, &orientation_error) &&
        position_error <= kPathPositionM && orientation_error <= kPathOrientationRad;
      if (std::isfinite(position_error) &&
          (!std::isfinite(record->max_position) || position_error > record->max_position))
      {
        record->max_position = position_error;
      }
      if (std::isfinite(orientation_error) &&
          (!std::isfinite(record->max_orientation) || orientation_error > record->max_orientation))
      {
        record->max_orientation = orientation_error;
      }
      if (!accurate)
      {
        ++record->accuracy_fail;
        ++record->collision_not_evaluated;
        failure.reason = "ACCURACY";
        record->failures.push_back(failure);
        continue;
      }
    }
    LocalScene & local = local_scene();
    std::vector<double> query(joints.data(), joints.data() + joints.size());
    local.state.setJointGroupPositions(g_shared->group, query);
    local.state.update();
    bool self_hit = false;
    bool coupon_hit = false;
    bool error = false;
    Counts scratch;
    apply_collision(&scratch, joints, &self_hit, &coupon_hit, &error);
    if (error)
    {
      ++record->error;
      ++record->collision_not_evaluated;
      failure.reason = "ERROR";
      record->failures.push_back(failure);
      continue;
    }
    if (self_hit)
    {
      ++record->self_hit;
      ++record->collision_not_evaluated;
      failure.reason = "SELF";
      collect_contacts(true, local.state, &failure);
      record->failures.push_back(failure);
      continue;
    }
    if (coupon_hit)
    {
      ++record->coupon_hit;
      failure.reason = "COUPON";
      collect_contacts(false, local.state, &failure);
      record->failures.push_back(failure);
    }
  }
  const bool counts_clear = record->samples == kSampleDenominator + 1 && record->nonfinite == 0 &&
    record->limit_fail == 0 && record->self_hit == 0 && record->coupon_hit == 0 &&
    record->collision_not_evaluated == 0 && record->error == 0;
  const bool accuracy_clear = transition || record->accuracy_fail == 0;
  record->status = counts_clear && accuracy_clear ? "PASS" : "FAIL";
}

void write_half(std::ostream & output, const HalfRecord & record)
{
  output << "{\"half\": " << record.half
         << ", \"samples\": " << record.samples
         << ", \"accuracy_applied\": " << (record.transition ? "false" : "true");
  if (record.transition)
  {
    output << ", \"max_position_error_m\": null, \"max_orientation_error_rad\": null, \"accuracy_fail\": null";
  }
  else
  {
    output << ", \"max_position_error_m\": ";
    write_number(output, record.max_position);
    output << ", \"max_orientation_error_rad\": ";
    write_number(output, record.max_orientation);
    output << ", \"accuracy_fail\": " << record.accuracy_fail;
  }
  output << ", \"nonfinite\": " << record.nonfinite
         << ", \"limit_fail\": " << record.limit_fail
         << ", \"self_hit\": " << record.self_hit
         << ", \"coupon_hit\": " << record.coupon_hit
         << ", \"collision_not_evaluated\": " << record.collision_not_evaluated
         << ", \"error\": " << record.error
         << ", \"endpoint_identity_checked\": " << (record.endpoint_checked ? "true" : "false")
         << ", \"endpoint_identity_max_rad\": ";
  write_number(output, record.endpoint_checked ? record.endpoint_identity_rad : std::numeric_limits<double>::quiet_NaN());
  output << ", \"endpoint_identity_in_pass\": false, \"status\": \"" << record.status << "\", \"failures\": [";
  for (std::size_t index = 0; index < record.failures.size(); ++index)
  {
    if (index)
    {
      output << ", ";
    }
    const SampleFailure & failure = record.failures[index];
    output << "{\"k\": " << failure.sample << ", \"t\": ";
    write_number(output, failure.parameter);
    output << ", \"reason\": \"" << failure.reason << "\", \"joints_rad\": [";
    for (int joint = 0; joint < failure.joints.size(); ++joint)
    {
      if (joint)
      {
        output << ", ";
      }
      write_number(output, failure.joints[joint]);
    }
    output << "], \"command_position_m\": ";
    if (!failure.position_applied)
    {
      output << "null";
    }
    else
    {
      output << '[';
      write_number(output, failure.position.x());
      output << ", ";
      write_number(output, failure.position.y());
      output << ", ";
      write_number(output, failure.position.z());
      output << ']';
    }
    output << ", \"contact_count\": " << failure.contact_count
           << ", \"contacts_stored\": " << failure.contacts.size()
           << ", \"contact_storage_limited\": "
           << (failure.contact_count > failure.contacts.size() || failure.contact_count >= 20 ? "true" : "false")
           << ", \"contacts\": [";
    for (std::size_t contact_index = 0; contact_index < failure.contacts.size(); ++contact_index)
    {
      if (contact_index)
      {
        output << ", ";
      }
      const ContactRecord & contact = failure.contacts[contact_index];
      output << "{\"body_1\": \"" << json_escape(contact.first) << "\", \"body_2\": \"" << json_escape(contact.second)
             << "\", \"position_m\": [";
      write_number(output, contact.x);
      output << ", ";
      write_number(output, contact.y);
      output << ", ";
      write_number(output, contact.z);
      output << "], \"depth_m\": ";
      write_number(output, contact.depth);
      output << '}';
    }
    output << "]}";
  }
  output << "]}";
}

int run_restore(
  const std::vector<std::vector<Waypoint>> & runs,
  const std::string & transitions_path,
  const std::string & inserts_path,
  const std::string & selected_path,
  int threads)
{
  const std::string selected_text = read_file(selected_path);
  std::vector<SelectedSlot> selected;
  std::string canonical;
  if (!parse_selected(selected_text, &selected, &canonical) || sha256_text(canonical) != kSelectedSha)
  {
    write_status_file("BLOCKED_INPUT_INTEGRITY", "SELECTED", "selected_hash");
    return 2;
  }
  std::vector<Rung> rungs;
  std::vector<EdgeKind> edges;
  if (!assemble_full(runs, transitions_path, inserts_path, &rungs, &edges) || selected.size() != rungs.size())
  {
    write_status_file("BLOCKED_INPUT_INTEGRITY", "SELECTED", "rung_count");
    return 2;
  }
  const int transition_ids[20] = {
    35, 70, 105, 140, 175, 210, 245, 281, 317, 353, 389, 425, 461, 496, 531, 566, 601, 636, 671, 706};
  for (std::size_t index = 0; index < rungs.size(); ++index)
  {
    const SelectedSlot & slot = selected[index];
    const Rung & rung = rungs[index];
    if (slot.rung != static_cast<int>(index) || slot.inserted != rung.inserted ||
        slot.sequence_id != rung.sequence_id || slot.from_sequence_id != rung.from_sequence_id ||
        slot.to_sequence_id != rung.to_sequence_id || slot.candidate_index != 224 ||
        rung.candidates.size() != 256)
    {
      write_status_file("BLOCKED_INPUT_INTEGRITY", "SELECTED", "slot_224");
      return 2;
    }
  }
  std::vector<std::string> filter_status(rungs.size(), "NOT_EVALUATED");
  std::vector<std::vector<int>> earlier_equal(rungs.size());
  std::string filter_result = "NOT_STARTED";
  bool filter_blocked = false;
  if (!begin_phase(kRestoreFilterS))
  {
    filter_result = "NOT_STARTED_INSUFFICIENT_TIME";
    filter_blocked = true;
  }
  else
  {
    filter_result = "IN_SURVIVORS";
    for (std::size_t index = 0; index < rungs.size(); ++index)
    {
      if (timed_out())
      {
        filter_result = "GRAPH_INCOMPLETE";
        filter_blocked = true;
        break;
      }
      filter_rung(&rungs[index], threads);
      if (g_shared->time_exceeded.load())
      {
        filter_result = "GRAPH_INCOMPLETE";
        filter_blocked = true;
        break;
      }
      const auto & survivors = rungs[index].survivors;
      const bool present = std::find(survivors.begin(), survivors.end(), static_cast<std::size_t>(224)) != survivors.end();
      if (!present)
      {
        filter_status[index] = slot_filter_reason(
          rungs[index].candidates[224], rungs[index].position, rungs[index].orientation);
        filter_result = "NOT_IN_SURVIVORS";
        filter_blocked = true;
        break;
      }
      filter_status[index] = "IN_SURVIVORS";
      for (std::size_t survivor : survivors)
      {
        if (survivor >= 224)
        {
          break;
        }
        if (same_vector(rungs[index].candidates[survivor].joints, rungs[index].candidates[224].joints))
        {
          earlier_equal[index].push_back(static_cast<int>(survivor));
        }
      }
    }
  }
  std::vector<HalfRecord> halves(edges.size());
  std::string inspection = filter_blocked ? "NOT_EVALUATED" : "NOT_STARTED";
  if (!filter_blocked)
  {
    if (!begin_phase(kRestoreInspectS))
    {
      inspection = "NOT_STARTED_INSUFFICIENT_TIME";
    }
    else
    {
      inspection = "DONE";
      for (std::size_t index = 0; index < edges.size(); ++index)
      {
        if (timed_out())
        {
          inspection = "NOT_EVALUATED";
          break;
        }
        halves[index].half = edges[index].half;
        inspect_half(
          rungs[index], rungs[index + 1], 224, 224, edges[index].transition, &halves[index]);
        if (halves[index].status == "NOT_EVALUATED")
        {
          inspection = "NOT_EVALUATED";
          break;
        }
      }
    }
  }
  int original_pass = 0;
  int original_fail = 0;
  int original_not_evaluated = 0;
  std::map<std::pair<int, int>, std::vector<std::size_t>> groups;
  for (std::size_t index = 0; index < edges.size(); ++index)
  {
    groups[{edges[index].original_from, edges[index].original_to}].push_back(index);
  }
  std::ofstream output(g_out_path);
  output << std::setprecision(17);
  output << "{\n  \"gate\": \"UR-ANALYTIC-IK-PROCESS-FULL-LAYERED-GRAPH-RESTORE\",\n"
         << "  \"new_search\": false,\n  \"candidate_replacement\": false,\n"
         << "  \"integrated_manifest\": \"NOT_ADOPTED\",\n  \"p022\": \"UNCHANGED\",\n"
         << "  \"prior_aggregate\": \"MAINTAINED\",\n  \"continuous_safety\": \"NOT_CLAIMED\",\n"
         << "  \"time_parameterization\": \"NOT_CLAIMED\",\n  \"physical_execution\": \"NOT_CLAIMED\",\n"
         << "  \"selected_sha256\": \"" << kSelectedSha << "\",\n"
         << "  \"filter_result\": \"" << filter_result << "\",\n"
         << "  \"inspection\": \"" << inspection << "\",\n  \"joints\": [";
  for (std::size_t index = 0; index < rungs.size(); ++index)
  {
    if (index)
    {
      output << ", ";
    }
    const Rung & rung = rungs[index];
    const bool transition = !rung.inserted &&
      std::find(std::begin(transition_ids), std::end(transition_ids), rung.sequence_id) != std::end(transition_ids);
    const char * file_name = rung.inserted ? "inserts.bin" : transition ? "transitions.bin" : "runs.bin";
    const char * file_hash = rung.inserted ? kInsertsSha : transition ? kTransitionsSha : kRunsSha;
    const Candidate & candidate = rung.candidates[224];
    output << "{\"rung\": " << index << ", \"file\": \"" << file_name << "\", \"file_sha256\": \"" << file_hash
           << "\", \"inserted\": " << (rung.inserted ? "true" : "false")
           << ", \"sequence_id\": " << rung.sequence_id
           << ", \"from_sequence_id\": " << rung.from_sequence_id
           << ", \"to_sequence_id\": " << rung.to_sequence_id
           << ", \"candidate_index\": 224, \"kinematic_class_id\": " << candidate.kinematic_class_id
           << ", \"source_code\": " << candidate.source_code
           << ", \"filter\": \"" << filter_status[index] << "\", \"earlier_equal_joints\": [";
    for (std::size_t equal_index = 0; equal_index < earlier_equal[index].size(); ++equal_index)
    {
      if (equal_index)
      {
        output << ", ";
      }
      output << earlier_equal[index][equal_index];
    }
    output << "], \"joints_rad\": [";
    for (int joint = 0; joint < candidate.joints.size(); ++joint)
    {
      if (joint)
      {
        output << ", ";
      }
      write_number(output, candidate.joints[joint]);
    }
    output << "]}";
  }
  output << "],\n  \"segments\": ";
  if (inspection != "DONE")
  {
    output << "\"NOT_EVALUATED\"";
  }
  else
  {
    output << '[';
    bool first_segment = true;
    for (const auto & group : groups)
    {
      if (!first_segment)
      {
        output << ", ";
      }
      first_segment = false;
      const bool transition = edges[group.second.front()].transition;
      bool pass = true;
      bool not_evaluated = false;
      for (std::size_t edge_index : group.second)
      {
        if (halves[edge_index].status == "NOT_EVALUATED")
        {
          not_evaluated = true;
        }
        if (halves[edge_index].status != "PASS")
        {
          pass = false;
        }
      }
      const std::string status = not_evaluated ? "NOT_EVALUATED" : pass ? "PASS" : "FAIL";
      if (status == "PASS")
      {
        ++original_pass;
      }
      else if (status == "FAIL")
      {
        ++original_fail;
      }
      else
      {
        ++original_not_evaluated;
      }
      output << "{\"original_from\": " << group.first.first << ", \"original_to\": " << group.first.second
             << ", \"kind\": \"" << (transition ? "TRANSITION" : "PROCESS") << "\", \"status\": \"" << status
             << "\", \"halves\": [";
      for (std::size_t half_index = 0; half_index < group.second.size(); ++half_index)
      {
        if (half_index)
        {
          output << ", ";
        }
        write_half(output, halves[group.second[half_index]]);
      }
      output << "]}";
    }
    output << ']';
  }
  const bool discrete_pass = filter_result == "IN_SURVIVORS" && inspection == "DONE" &&
    original_pass == 740 && original_fail == 0 && original_not_evaluated == 0;
  output << ",\n  \"original_pass\": " << original_pass
         << ",\n  \"original_fail\": " << original_fail
         << ",\n  \"original_not_evaluated\": " << original_not_evaluated
         << ",\n  \"discrete_pass\": " << (discrete_pass ? "true" : "false")
         << ",\n  \"conclusion\": \""
         << (discrete_pass ? "저장 관절열의 원본 740구간에 대한 고정 표본 이산 검사 통과" :
              original_fail > 0 ? "고정 표본 이산 검사에서 구간 실패가 있음" : "NOT_EVALUATED")
         << "\",\n  \"process_elapsed_s\": ";
  write_number(output, process_elapsed());
  output << ",\n  \"complete\": true\n}\n";
  if (filter_result == "NOT_IN_SURVIVORS" || filter_result == "GRAPH_INCOMPLETE" ||
      filter_result == "NOT_STARTED_INSUFFICIENT_TIME" || inspection == "NOT_EVALUATED" ||
      inspection == "NOT_STARTED_INSUFFICIENT_TIME")
  {
    return 3;
  }
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  g_process_start = std::chrono::steady_clock::now();
  g_overall_s = 1.0e9;
  g_phase_wall_s = kInternalWallS;
  try
  {
    if (argc < 8)
    {
      std::cerr << "usage: connect_collision precheck|run ...\n";
      return 2;
    }
    const std::string mode = argv[1];
    int run_id = -1;
    int argument = 2;
    if (mode == "run")
    {
      run_id = std::stoi(argv[2]);
      argument = 3;
      bool allowed = false;
      for (int target : kTargetRuns)
      {
        if (target == run_id)
        {
          allowed = true;
        }
      }
      if (!allowed)
      {
        std::cerr << "BLOCKED_INPUT_INTEGRITY run\n";
        return 2;
      }
    }
    else if (mode == "boundary")
    {
      g_overall_s = kBoundaryOverallS;
    }
    else if (mode == "fullgraph")
    {
      g_overall_s = kFullOverallS;
    }
    else if (mode == "restorecheck")
    {
      g_overall_s = kRestoreOverallS;
    }
    else if (mode != "precheck")
    {
      std::cerr << "EXECUTION_ERROR mode\n";
      return 4;
    }
    const int tail = mode == "fullgraph" ? 8 : mode == "restorecheck" ? 9 : 7;
    if (argc != argument + tail)
    {
      std::cerr << "EXECUTION_ERROR arguments\n";
      return 4;
    }
    const char * urdf_path = argv[argument];
    const char * srdf_path = argv[argument + 1];
    const char * runs_path = argv[argument + 2];
    const char * mesh_path = argv[argument + 3];
    const char * acceptance_path = argv[argument + 4];
    const char * inserts_path = argv[argument + 5];
    const char * transitions_path = mode == "boundary" || mode == "fullgraph" || mode == "restorecheck" ? argv[argument + 5] : "";
    const char * full_inserts_path = mode == "fullgraph" || mode == "restorecheck" ? argv[argument + 6] : "";
    const char * selected_path = mode == "restorecheck" ? argv[argument + 7] : "";
    g_out_path = argv[argument + tail - 1];
    if (!address_limit_ok())
    {
      write_status_file("EXECUTION_ERROR", "SETUP", "address_limit");
      return 4;
    }
    if (sha256_file(urdf_path) != kUrdfSha || sha256_file(srdf_path) != kSrdfSha ||
        sha256_file(runs_path) != kRunsSha || sha256_file(mesh_path) != kMeshSha ||
        sha256_file(acceptance_path) != kAcceptanceSha ||
        ((mode == "boundary" || mode == "fullgraph" || mode == "restorecheck") && sha256_file(transitions_path) != kTransitionsSha) ||
        (mode == "restorecheck" && sha256_file(full_inserts_path) != kInsertsSha))
    {
      write_status_file("EXECUTION_ERROR", "SETUP", "input_hash");
      return 2;
    }
    const std::string acceptance = read_file(acceptance_path);
    if (acceptance.find("\"accepted\": true") == std::string::npos)
    {
      write_status_file("EXECUTION_ERROR", "SETUP", "acceptance");
      return 2;
    }
    Shared shared;
    g_shared = &shared;
    if (!load_coupon(mesh_path))
    {
      write_status_file("EXECUTION_ERROR", "SETUP", "mesh");
      return 2;
    }
    std::vector<std::vector<Waypoint>> runs;
    std::map<int, std::vector<Insert>> inserts;
    const bool inputs_ok = load_runs(runs_path, &runs) &&
      (mode == "boundary" || mode == "fullgraph" || mode == "restorecheck" || load_inserts(inserts_path, &inserts));
    if (!inputs_ok)
    {
      write_status_file("EXECUTION_ERROR", "SETUP", "candidates");
      return 2;
    }
    rclcpp::init(argc, argv);
    auto urdf = urdf::parseURDF(read_file(urdf_path));
    auto srdf = std::make_shared<srdf::Model>();
    if (!urdf || !srdf->initString(*urdf, read_file(srdf_path)))
    {
      write_status_file("EXECUTION_ERROR", "SETUP", "robot");
      return 4;
    }
    std::string mesh_error;
    if (!robot_mesh_hashes_ok(*urdf, &mesh_error))
    {
      write_status_file("EXECUTION_ERROR", "SETUP", mesh_error);
      return 2;
    }
    shared.model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    shared.group = shared.model->getJointModelGroup("ur_manipulator");
    if (shared.group == nullptr || shared.group->getActiveJointModelNames().size() != 6)
    {
      write_status_file("EXECUTION_ERROR", "SETUP", "group");
      return 4;
    }
    shared.limits.resize(6, 2);
    for (int index = 0; index < 6; ++index)
    {
      if (shared.group->getActiveJointModelNames()[static_cast<std::size_t>(index)] != kJointNames[index])
      {
        write_status_file("EXECUTION_ERROR", "SETUP", "joints");
        return 4;
      }
      const auto & bounds = shared.group->getActiveJointModels()[static_cast<std::size_t>(index)]->getVariableBounds();
      if (bounds.size() != 1 || !bounds[0].position_bounded_)
      {
        write_status_file("EXECUTION_ERROR", "SETUP", "limits");
        return 4;
      }
      shared.limits(index, 0) = bounds[0].min_position_;
      shared.limits(index, 1) = bounds[0].max_position_;
    }
    shared.acm = planning_scene::PlanningScene(shared.model).getAllowedCollisionMatrix();
    shared.request.group_name = "ur_manipulator";
    shared.request.pad_environment_collisions = true;
    shared.request.pad_self_collisions = false;
    shared.request.contacts = true;
    shared.request.max_contacts = 20;
    shared.request.max_contacts_per_pair = 4;
    shared.request.distance = false;
    shared.started = std::chrono::steady_clock::now();
    const int threads = std::min(24, std::max(1, static_cast<int>(std::thread::hardware_concurrency())));
    if (mode == "boundary")
    {
      if (!(kPrepS - process_elapsed() >= 0.0) || process_elapsed() > kPrepS)
      {
        write_status_file("NOT_STARTED_INSUFFICIENT_TIME", "PREP", "prep_cap");
        rclcpp::shutdown();
        return 3;
      }
      const int status = run_boundary(runs, transitions_path, threads);
      rclcpp::shutdown();
      return status;
    }
    if (mode == "fullgraph")
    {
      const int status = run_full(runs, transitions_path, full_inserts_path, threads);
      rclcpp::shutdown();
      return status;
    }
    if (mode == "restorecheck")
    {
      const int status = run_restore(runs, transitions_path, full_inserts_path, selected_path, threads);
      rclcpp::shutdown();
      return status;
    }
    const int status = mode == "precheck" ? precheck(runs, threads) :
      search_run(runs[static_cast<std::size_t>(run_id)], run_id == 6 || run_id == 11 ?
        &inserts[run_id] : nullptr, run_id, threads);
    rclcpp::shutdown();
    return status;
  }
  catch (const std::bad_alloc &)
  {
    write_status_file("TIME_OR_RESOURCE_LIMIT", "MEMORY", "bad_alloc");
    std::cerr << "RESOURCE_LIMIT bad_alloc\n";
    return 3;
  }
  catch (const std::exception & error)
  {
    write_status_file("EXECUTION_ERROR", "EXCEPTION", "exception");
    std::cerr << "EXECUTION_ERROR " << error.what() << "\n";
    return 4;
  }
}
