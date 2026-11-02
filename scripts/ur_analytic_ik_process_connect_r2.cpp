// UR-ANALYTIC-IK-PROCESS-CONNECT-R2. run 6–12만 정확성 edge로 다시 탐색한다.
// 첫 탐색 결과는 쓰지 않는다. 시작 관절은 자유다.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <new>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <omp.h>

#include <sys/resource.h>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <descartes_light/core/waypoint_sampler.h>
#include <descartes_light/edge_evaluators/compound_edge_evaluator.h>
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>
#include <descartes_light/solvers/ladder_graph/ladder_graph_solver.h>

namespace
{
const char * kJointNames[6] = {
  "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
const int kRunCount = 21;
const int kCandidateCount = 256;
const int kSampleDenominator = 100;
const double kTranslationMaxM = 0.001;
const double kRotationMaxRad = 0.01;
const rlim_t kAddressSpaceBytes = 8ULL * 1024ULL * 1024ULL * 1024ULL;
const int kPrecheckPairs[10][2] = {
  {0, 0}, {0, 1}, {1, 0}, {5, 7}, {7, 5},
  {10, 20}, {3, 255}, {100, 200}, {128, 64}, {255, 255}};

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

struct SampleStats
{
  double max_position = -1.0;
  double max_orientation = -1.0;
  int max_position_start = -1;
  int max_position_end = -1;
  double max_position_t = std::numeric_limits<double>::quiet_NaN();
  int max_orientation_start = -1;
  int max_orientation_end = -1;
  double max_orientation_t = std::numeric_limits<double>::quiet_NaN();
  bool within = true;
  int fk_count = 0;
};

double rotation_error_rad(const Eigen::Matrix3d & target, const Eigen::Matrix3d & actual);
Eigen::Isometry3d tool_in_base(const moveit::core::RobotState & state);

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

class SampledAccuracyEdgeEvaluator : public descartes_light::EdgeEvaluator<double>
{
public:
  SampledAccuracyEdgeEvaluator(
    Eigen::Vector3d start_position,
    Eigen::Vector3d end_position,
    Eigen::Quaterniond start_orientation,
    Eigen::Quaterniond end_orientation,
    moveit::core::RobotModelConstPtr model,
    const moveit::core::JointModelGroup * group)
  : start_position_(std::move(start_position)),
    end_position_(std::move(end_position)),
    start_orientation_(std::move(start_orientation)),
    end_orientation_(std::move(end_orientation)),
    model_(std::move(model)),
    group_(group)
  {
  }

  std::pair<bool, double> evaluate(
    const descartes_light::State<double> & start,
    const descartes_light::State<double> & end) const override
  {
    int checked = 0;
    bool nonfinite = false;
    const bool accepted = accept_edge(start.values, end.values, &checked, &nonfinite);
    fk_checks_.fetch_add(static_cast<std::uint64_t>(checked), std::memory_order_relaxed);
    if (nonfinite)
    {
      nonfinite_rejects_.fetch_add(1, std::memory_order_relaxed);
    }
    if (!accepted)
    {
      if (checked < kSampleDenominator + 1)
      {
        early_rejects_.fetch_add(1, std::memory_order_relaxed);
      }
      return std::make_pair(false, 0.0);
    }
    return std::make_pair(true, 0.0);
  }

  std::uint64_t fk_checks() const
  {
    return fk_checks_.load(std::memory_order_relaxed);
  }

  std::uint64_t early_rejects() const
  {
    return early_rejects_.load(std::memory_order_relaxed);
  }

  std::uint64_t nonfinite_rejects() const
  {
    return nonfinite_rejects_.load(std::memory_order_relaxed);
  }

private:
  bool accept_edge(
    const Eigen::VectorXd & start_joints,
    const Eigen::VectorXd & end_joints,
    int * checked,
    bool * nonfinite) const
  {
    if (!start_joints.allFinite() || !end_joints.allFinite())
    {
      *nonfinite = true;
      return false;
    }
    const int order[kSampleDenominator + 1] = {
      50, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
      10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
      20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
      30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
      40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
      51, 52, 53, 54, 55, 56, 57, 58, 59,
      60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
      70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
      80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
      90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100};
    for (int sample_index : order)
    {
      double position_error = 0.0;
      double orientation_error = 0.0;
      const bool finite = sample_error(
        start_joints, end_joints, sample_index, &position_error, &orientation_error);
      ++(*checked);
      if (!finite)
      {
        *nonfinite = true;
        return false;
      }
      if (!(position_error <= kTranslationMaxM) || !(orientation_error <= kRotationMaxRad))
      {
        return false;
      }
    }
    return true;
  }

  bool sample_error(
    const Eigen::VectorXd & start_joints,
    const Eigen::VectorXd & end_joints,
    int sample_index,
    double * position_error,
    double * orientation_error) const
  {
    const double fraction = static_cast<double>(sample_index) / static_cast<double>(kSampleDenominator);
    const Eigen::VectorXd joints = (1.0 - fraction) * start_joints + fraction * end_joints;
    if (!joints.allFinite())
    {
      return false;
    }
    std::vector<double> query(6);
    for (int index = 0; index < 6; ++index)
    {
      query[static_cast<std::size_t>(index)] = joints[index];
    }
    moveit::core::RobotState & state = thread_state();
    state.setJointGroupPositions(group_, query);
    state.update();
    const Eigen::Isometry3d actual = tool_in_base(state);
    const Eigen::Vector3d target_position = (1.0 - fraction) * start_position_ + fraction * end_position_;
    const Eigen::Quaterniond target_orientation = start_orientation_.slerp(fraction, end_orientation_);
    const Eigen::Matrix3d target_matrix = target_orientation.toRotationMatrix();
    if (!target_position.allFinite() || !target_matrix.allFinite() ||
        !actual.translation().allFinite() || !actual.rotation().allFinite())
    {
      return false;
    }
    *position_error = (actual.translation() - target_position).norm();
    *orientation_error = rotation_error_rad(target_matrix, actual.rotation());
    return std::isfinite(*position_error) && std::isfinite(*orientation_error);
  }

  moveit::core::RobotState & thread_state() const
  {
    thread_local std::unique_ptr<moveit::core::RobotState> state;
    if (!state)
    {
      state = std::make_unique<moveit::core::RobotState>(model_);
    }
    return *state;
  }

  const Eigen::Vector3d start_position_;
  const Eigen::Vector3d end_position_;
  const Eigen::Quaterniond start_orientation_;
  const Eigen::Quaterniond end_orientation_;
  const moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup * group_;
  mutable std::atomic<std::uint64_t> fk_checks_{0};
  mutable std::atomic<std::uint64_t> early_rejects_{0};
  mutable std::atomic<std::uint64_t> nonfinite_rejects_{0};
};

class FiniteCostGate : public descartes_light::EdgeEvaluator<double>
{
public:
  FiniteCostGate(
    descartes_light::EdgeEvaluator<double>::ConstPtr accuracy,
    descartes_light::EdgeEvaluator<double>::ConstPtr distance)
  {
    auto compound = std::make_shared<descartes_light::CompoundEdgeEvaluator<double>>();
    compound->evaluators.push_back(std::move(accuracy));
    compound->evaluators.push_back(std::move(distance));
    compound_ = compound;
  }

  std::pair<bool, double> evaluate(
    const descartes_light::State<double> & start,
    const descartes_light::State<double> & end) const override
  {
    edge_evaluations_.fetch_add(1, std::memory_order_relaxed);
    const std::pair<bool, double> result = compound_->evaluate(start, end);
    if (!result.first || !std::isfinite(result.second))
    {
      return std::make_pair(false, 0.0);
    }
    return result;
  }

  std::uint64_t edge_evaluations() const
  {
    return edge_evaluations_.load(std::memory_order_relaxed);
  }

private:
  std::shared_ptr<descartes_light::CompoundEdgeEvaluator<double>> compound_;
  mutable std::atomic<std::uint64_t> edge_evaluations_{0};
};

std::string read_file(const std::string & path)
{
  std::ifstream input(path);
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
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

double rotation_error_rad(const Eigen::Matrix3d & target, const Eigen::Matrix3d & actual)
{
  const Eigen::Matrix3d relative = target.transpose() * actual;
  double cosine = (relative.trace() - 1.0) * 0.5;
  if (cosine > 1.0)
  {
    cosine = 1.0;
  }
  if (cosine < -1.0)
  {
    cosine = -1.0;
  }
  return std::acos(cosine);
}

Eigen::Isometry3d tool_in_base(const moveit::core::RobotState & state)
{
  return state.getGlobalLinkTransform("base_link").inverse() * state.getGlobalLinkTransform("tool0");
}

template <typename T>
T read_pod(std::istream & input)
{
  T value{};
  input.read(reinterpret_cast<char *>(&value), sizeof(T));
  return value;
}

bool load_runs(const std::string & path, std::vector<std::vector<Waypoint>> & runs)
{
  std::ifstream input(path, std::ios::binary);
  char magic[5] = {};
  input.read(magic, 5);
  if (!input || std::string(magic, 5) != "URPC1")
  {
    return false;
  }
  const auto run_count = read_pod<std::uint32_t>(input);
  if (run_count != static_cast<std::uint32_t>(kRunCount))
  {
    return false;
  }
  runs.resize(kRunCount);
  int process_count = 0;
  for (int run_id = 0; run_id < kRunCount; ++run_id)
  {
    const auto stored_run = read_pod<std::int32_t>(input);
    const auto waypoint_count = read_pod<std::uint32_t>(input);
    if (!input || stored_run != run_id || waypoint_count == 0)
    {
      return false;
    }
    runs[static_cast<std::size_t>(run_id)].resize(waypoint_count);
    for (std::uint32_t waypoint_index = 0; waypoint_index < waypoint_count; ++waypoint_index)
    {
      Waypoint & waypoint = runs[static_cast<std::size_t>(run_id)][waypoint_index];
      waypoint.sequence_id = read_pod<std::int32_t>(input);
      double position[3];
      double orientation[4];
      input.read(reinterpret_cast<char *>(position), sizeof(position));
      input.read(reinterpret_cast<char *>(orientation), sizeof(orientation));
      waypoint.position = Eigen::Vector3d(position[0], position[1], position[2]);
      waypoint.orientation = Eigen::Quaterniond(
        orientation[3], orientation[0], orientation[1], orientation[2]);
      const auto candidate_count = read_pod<std::uint32_t>(input);
      if (!input || candidate_count != static_cast<std::uint32_t>(kCandidateCount))
      {
        return false;
      }
      waypoint.candidates.resize(kCandidateCount);
      for (int candidate_index = 0; candidate_index < kCandidateCount; ++candidate_index)
      {
        Candidate & candidate = waypoint.candidates[static_cast<std::size_t>(candidate_index)];
        candidate.kinematic_class_id = read_pod<std::int32_t>(input);
        candidate.source_code = read_pod<std::uint8_t>(input);
        double joints[6];
        input.read(reinterpret_cast<char *>(joints), sizeof(joints));
        if (!input || (candidate.source_code != 0 && candidate.source_code != 1) ||
            !std::isfinite(joints[0]) || !std::isfinite(joints[1]) || !std::isfinite(joints[2]) ||
            !std::isfinite(joints[3]) || !std::isfinite(joints[4]) || !std::isfinite(joints[5]))
        {
          return false;
        }
        candidate.joints = Eigen::VectorXd(6);
        for (int joint_index = 0; joint_index < 6; ++joint_index)
        {
          candidate.joints[joint_index] = joints[joint_index];
        }
      }
      if (!waypoint.position.allFinite() || !std::isfinite(waypoint.orientation.norm()) ||
          !(waypoint.orientation.norm() > 0.0))
      {
        return false;
      }
      ++process_count;
    }
  }
  return process_count == 721 && input && input.peek() == EOF;
}

const char * source_name(int source_code)
{
  return source_code == 0 ? "solver" : "redundant_2pi";
}

void install_address_limit()
{
  struct rlimit limit;
  limit.rlim_cur = kAddressSpaceBytes;
  limit.rlim_max = kAddressSpaceBytes;
  if (setrlimit(RLIMIT_AS, &limit) != 0)
  {
    std::cerr << "EXECUTION_ERROR setrlimit\n";
    std::_Exit(4);
  }
}

std::shared_ptr<FiniteCostGate> make_gate(
  const Waypoint & start,
  const Waypoint & end,
  const moveit::core::RobotModelConstPtr & model,
  const moveit::core::JointModelGroup * group,
  const descartes_light::EdgeEvaluator<double>::ConstPtr & distance,
  std::shared_ptr<SampledAccuracyEdgeEvaluator> * accuracy_out)
{
  auto accuracy = std::make_shared<SampledAccuracyEdgeEvaluator>(
    start.position, end.position, start.orientation, end.orientation, model, group);
  *accuracy_out = accuracy;
  return std::make_shared<FiniteCostGate>(accuracy, distance);
}

bool verify_path(
  const std::vector<Waypoint> & waypoints,
  const std::vector<std::size_t> & chosen,
  moveit::core::RobotState & state,
  const moveit::core::JointModelGroup * group,
  SampleStats * stats)
{
  stats->within = true;
  for (std::size_t segment = 0; segment + 1 < waypoints.size(); ++segment)
  {
    const Waypoint & start = waypoints[segment];
    const Waypoint & end = waypoints[segment + 1];
    const Eigen::VectorXd & start_joints = start.candidates[chosen[segment]].joints;
    const Eigen::VectorXd & end_joints = end.candidates[chosen[segment + 1]].joints;
    for (int sample_index = 0; sample_index <= kSampleDenominator; ++sample_index)
    {
      const double fraction = static_cast<double>(sample_index) / static_cast<double>(kSampleDenominator);
      const Eigen::VectorXd joints = (1.0 - fraction) * start_joints + fraction * end_joints;
      ++stats->fk_count;
      if (!joints.allFinite())
      {
        stats->within = false;
        continue;
      }
      std::vector<double> query(6);
      for (int index = 0; index < 6; ++index)
      {
        query[static_cast<std::size_t>(index)] = joints[index];
      }
      state.setJointGroupPositions(group, query);
      state.update();
      const Eigen::Isometry3d actual = tool_in_base(state);
      const Eigen::Vector3d target_position = (1.0 - fraction) * start.position + fraction * end.position;
      const Eigen::Quaterniond target_orientation = start.orientation.slerp(fraction, end.orientation);
      const Eigen::Matrix3d target_matrix = target_orientation.toRotationMatrix();
      if (!target_position.allFinite() || !target_matrix.allFinite() ||
          !actual.translation().allFinite() || !actual.rotation().allFinite())
      {
        stats->within = false;
        continue;
      }
      const double position_error = (actual.translation() - target_position).norm();
      const double orientation_error = rotation_error_rad(target_matrix, actual.rotation());
      if (!std::isfinite(position_error) || !std::isfinite(orientation_error) ||
          !(position_error <= kTranslationMaxM) || !(orientation_error <= kRotationMaxRad))
      {
        stats->within = false;
      }
      if (std::isfinite(position_error) && position_error > stats->max_position)
      {
        stats->max_position = position_error;
        stats->max_position_start = start.sequence_id;
        stats->max_position_end = end.sequence_id;
        stats->max_position_t = fraction;
      }
      if (std::isfinite(orientation_error) && orientation_error > stats->max_orientation)
      {
        stats->max_orientation = orientation_error;
        stats->max_orientation_start = start.sequence_id;
        stats->max_orientation_end = end.sequence_id;
        stats->max_orientation_t = fraction;
      }
    }
  }
  return stats->within;
}

int precheck(
  const std::vector<std::vector<Waypoint>> & runs,
  const moveit::core::RobotModelConstPtr & model,
  const moveit::core::JointModelGroup * group,
  int threads,
  const std::string & out_path)
{
  const std::vector<Waypoint> & waypoints = runs[6];
  auto distance = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<double>>();
  std::shared_ptr<SampledAccuracyEdgeEvaluator> accuracy;
  auto gate = make_gate(waypoints[0], waypoints[1], model, group, distance, &accuracy);
  std::vector<descartes_light::State<double>::ConstPtr> starts;
  std::vector<descartes_light::State<double>::ConstPtr> ends;
  for (const auto & pair : kPrecheckPairs)
  {
    starts.push_back(std::make_shared<descartes_light::State<double>>(
      waypoints[0].candidates[static_cast<std::size_t>(pair[0])].joints));
    ends.push_back(std::make_shared<descartes_light::State<double>>(
      waypoints[1].candidates[static_cast<std::size_t>(pair[1])].joints));
  }
  std::vector<std::pair<bool, double>> single(starts.size());
  for (std::size_t index = 0; index < starts.size(); ++index)
  {
    single[index] = gate->evaluate(*starts[index], *ends[index]);
  }
  std::vector<std::pair<bool, double>> parallel(starts.size());
#pragma omp parallel for num_threads(threads) schedule(static)
  for (int index = 0; index < static_cast<int>(starts.size()); ++index)
  {
    parallel[static_cast<std::size_t>(index)] = gate->evaluate(
      *starts[static_cast<std::size_t>(index)], *ends[static_cast<std::size_t>(index)]);
  }
  bool matched = true;
  for (std::size_t index = 0; index < single.size(); ++index)
  {
    if (single[index].first != parallel[index].first ||
        !(std::isfinite(single[index].second) && single[index].second == parallel[index].second))
    {
      matched = false;
    }
  }
  std::ofstream output(out_path);
  output << std::setprecision(17);
  output << "{\n  \"gate\": \"UR-ANALYTIC-IK-PROCESS-CONNECT-R2\",\n"
         << "  \"role\": \"THREAD_PRECHECK\",\n"
         << "  \"threads\": " << threads << ",\n"
         << "  \"precheck_match\": " << (matched ? "true" : "false") << ",\n"
         << "  \"edges\": [\n";
  for (std::size_t index = 0; index < single.size(); ++index)
  {
    if (index)
    {
      output << ",\n";
    }
    output << "    {\"from\": " << kPrecheckPairs[index][0]
           << ", \"to\": " << kPrecheckPairs[index][1]
           << ", \"single_valid\": " << (single[index].first ? "true" : "false")
           << ", \"multi_valid\": " << (parallel[index].first ? "true" : "false")
           << ", \"single_cost\": ";
    write_number(output, single[index].second);
    output << ", \"multi_cost\": ";
    write_number(output, parallel[index].second);
    output << "}";
  }
  output << "\n  ],\n  \"complete\": true\n}\n";
  std::cout << "PRECHECK " << (matched ? "MATCH" : "MISMATCH") << "\n";
  return matched ? 0 : 5;
}

int search_run(
  const std::vector<Waypoint> & waypoints,
  int run_id,
  const moveit::core::RobotModelConstPtr & model,
  const moveit::core::JointModelGroup * group,
  int threads,
  const std::string & out_path)
{
  const auto started = std::chrono::steady_clock::now();
  auto distance = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<double>>();
  std::vector<std::vector<descartes_light::State<double>::ConstPtr>> states(waypoints.size());
  std::vector<descartes_light::WaypointSampler<double>::ConstPtr> samplers;
  std::vector<descartes_light::EdgeEvaluator<double>::ConstPtr> gates;
  std::vector<std::shared_ptr<FiniteCostGate>> gate_owners;
  std::vector<std::shared_ptr<SampledAccuracyEdgeEvaluator>> accuracies;
  for (std::size_t waypoint_index = 0; waypoint_index < waypoints.size(); ++waypoint_index)
  {
    for (const Candidate & candidate : waypoints[waypoint_index].candidates)
    {
      states[waypoint_index].push_back(
        std::make_shared<descartes_light::State<double>>(candidate.joints));
    }
    samplers.push_back(std::make_shared<RecordedJointSampler>(states[waypoint_index]));
  }
  for (std::size_t segment = 0; segment + 1 < waypoints.size(); ++segment)
  {
    std::shared_ptr<SampledAccuracyEdgeEvaluator> accuracy;
    auto gate = make_gate(
      waypoints[segment], waypoints[segment + 1], model, group, distance, &accuracy);
    accuracies.push_back(accuracy);
    gate_owners.push_back(gate);
    gates.push_back(gate);
  }
  descartes_light::LadderGraphSolver<double> solver(threads);
  const descartes_light::BuildStatus built = solver.build(samplers, gates, {});
  std::string search_status = "NO_PATH";
  std::string accuracy_status = "NOT_EVALUATED";
  double ladder_cost = std::numeric_limits<double>::quiet_NaN();
  std::vector<std::size_t> chosen;
  bool have_path = false;
  if (built)
  {
    try
    {
      const descartes_light::SearchResult<double> result = solver.search();
      if (result.trajectory.size() != waypoints.size() || !std::isfinite(result.cost))
      {
        std::cerr << "EXECUTION_ERROR path\n";
        return 4;
      }
      chosen.resize(waypoints.size());
      for (std::size_t waypoint_index = 0; waypoint_index < waypoints.size(); ++waypoint_index)
      {
        const auto & found = result.trajectory[waypoint_index];
        const auto match = std::find(
          states[waypoint_index].begin(), states[waypoint_index].end(), found);
        if (match == states[waypoint_index].end())
        {
          std::cerr << "EXECUTION_ERROR identity\n";
          return 4;
        }
        chosen[waypoint_index] = static_cast<std::size_t>(match - states[waypoint_index].begin());
        const Eigen::VectorXd & stored = waypoints[waypoint_index].candidates[chosen[waypoint_index]].joints;
        if (stored.size() != found->values.size() || stored != found->values)
        {
          std::cerr << "EXECUTION_ERROR joints\n";
          return 4;
        }
      }
      ladder_cost = result.cost;
      have_path = true;
    }
    catch (const std::runtime_error &)
    {
      have_path = false;
    }
  }
  SampleStats stats;
  moveit::core::RobotState verification_state(model);
  if (have_path)
  {
    const bool verified = verify_path(waypoints, chosen, verification_state, group, &stats);
    if (!verified)
    {
      search_status = "CHECK_MISMATCH";
      have_path = false;
    }
    else
    {
      search_status = "FOUND";
      accuracy_status = "SAMPLED_ACCURACY_PASS";
    }
  }
  std::uint64_t edge_count = 0;
  std::uint64_t early_count = 0;
  std::uint64_t fk_count = 0;
  std::uint64_t nonfinite_count = 0;
  for (const auto & gate : gate_owners)
  {
    edge_count += gate->edge_evaluations();
  }
  for (const auto & accuracy : accuracies)
  {
    early_count += accuracy->early_rejects();
    fk_count += accuracy->fk_checks();
    nonfinite_count += accuracy->nonfinite_rejects();
  }
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count();
  std::ofstream output(out_path);
  output << std::setprecision(17);
  output << "{\n  \"gate\": \"UR-ANALYTIC-IK-PROCESS-CONNECT-R2\",\n"
         << "  \"run_id\": " << run_id << ",\n"
         << "  \"search\": \"" << search_status << "\",\n"
         << "  \"process_accuracy\": \"" << accuracy_status << "\",\n"
         << "  \"termination_reason\": \"COMPLETED\",\n"
         << "  \"collision\": \"NOT_EVALUATED\",\n"
         << "  \"time_parameterization\": \"NOT_EVALUATED\",\n"
         << "  \"result_role\": \""
         << (search_status == "FOUND" ? "GEOMETRIC_CANDIDATE_PATH" : "NONE") << "\",\n"
         << "  \"threads\": " << threads << ",\n"
         << "  \"edge_evaluation_count\": " << edge_count << ",\n"
         << "  \"early_reject_count\": " << early_count << ",\n"
         << "  \"fk_check_count\": " << fk_count << ",\n"
         << "  \"nonfinite_reject_count\": " << nonfinite_count << ",\n"
         << "  \"verification_fk_count\": " << stats.fk_count << ",\n"
         << "  \"elapsed_s\": ";
  write_number(output, elapsed);
  output << ",\n  \"ladder_cost\": ";
  write_number(output, ladder_cost);
  output << ",\n  \"max_position_error_m\": ";
  write_number(output, stats.max_position);
  output << ",\n  \"max_position_sequence_ids\": [" << stats.max_position_start
         << ", " << stats.max_position_end << "],\n  \"max_position_t\": ";
  write_number(output, stats.max_position_t);
  output << ",\n  \"max_orientation_error_rad\": ";
  write_number(output, stats.max_orientation);
  output << ",\n  \"max_orientation_sequence_ids\": [" << stats.max_orientation_start
         << ", " << stats.max_orientation_end << "],\n  \"max_orientation_t\": ";
  write_number(output, stats.max_orientation_t);
  output << ",\n  \"selected\": [";
  if (search_status == "FOUND")
  {
    for (std::size_t waypoint_index = 0; waypoint_index < waypoints.size(); ++waypoint_index)
    {
      if (waypoint_index)
      {
        output << ", ";
      }
      const Candidate & candidate = waypoints[waypoint_index].candidates[chosen[waypoint_index]];
      output << "{\"sequence_id\": " << waypoints[waypoint_index].sequence_id
             << ", \"candidate_index\": " << chosen[waypoint_index]
             << ", \"source\": \"" << source_name(candidate.source_code)
             << "\", \"kinematic_class_id\": " << candidate.kinematic_class_id
             << ", \"joints_rad\": [";
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
  output << "],\n  \"complete\": true\n}\n";
  std::cout << "RUN " << run_id << " " << search_status << " " << accuracy_status
            << " edges " << edge_count << " early " << early_count
            << " fk " << fk_count << " elapsed " << elapsed << "\n";
  return search_status == "CHECK_MISMATCH" ? 6 : 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  try
  {
    install_address_limit();
    if (argc < 6)
    {
      std::cerr << "usage: ur_analytic_ik_process_connect_r2 precheck|run urdf srdf runs.bin out.json\n";
      return 2;
    }
    const std::string mode = argv[1];
    int run_id = -1;
    const char * urdf_path = nullptr;
    const char * srdf_path = nullptr;
    const char * runs_path = nullptr;
    const char * out_path = nullptr;
    if (mode == "precheck")
    {
      urdf_path = argv[2];
      srdf_path = argv[3];
      runs_path = argv[4];
      out_path = argv[5];
    }
    else if (mode == "run")
    {
      if (argc != 7)
      {
        std::cerr << "usage: ur_analytic_ik_process_connect_r2 run ID urdf srdf runs.bin out.json\n";
        return 2;
      }
      run_id = std::stoi(argv[2]);
      urdf_path = argv[3];
      srdf_path = argv[4];
      runs_path = argv[5];
      out_path = argv[6];
      if (run_id < 6 || run_id > 12)
      {
        std::cerr << "BLOCKED_INPUT_INTEGRITY run\n";
        return 2;
      }
    }
    else
    {
      std::cerr << "EXECUTION_ERROR mode\n";
      return 4;
    }
    std::vector<std::vector<Waypoint>> runs;
    if (!load_runs(runs_path, runs))
    {
      std::cerr << "BLOCKED_INPUT_INTEGRITY runs\n";
      return 2;
    }
    rclcpp::init(argc, argv);
    auto urdf = urdf::parseURDF(read_file(urdf_path));
    auto srdf = std::make_shared<srdf::Model>();
    if (!urdf || !srdf->initString(*urdf, read_file(srdf_path)))
    {
      std::cerr << "EXECUTION_ERROR robot\n";
      return 4;
    }
    auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    auto * group = model->getJointModelGroup("ur_manipulator");
    if (group == nullptr || model->getLinkModel("base_link") == nullptr ||
        model->getLinkModel("tool0") == nullptr)
    {
      std::cerr << "EXECUTION_ERROR robot\n";
      return 4;
    }
    const auto joint_names = group->getActiveJointModelNames();
    if (joint_names.size() != 6)
    {
      std::cerr << "EXECUTION_ERROR joints\n";
      return 4;
    }
    for (int index = 0; index < 6; ++index)
    {
      if (joint_names[static_cast<std::size_t>(index)] != kJointNames[index])
      {
        std::cerr << "EXECUTION_ERROR joints\n";
        return 4;
      }
    }
    const int threads = std::max(1, static_cast<int>(std::thread::hardware_concurrency()));
    int status = 0;
    if (mode == "precheck")
    {
      status = precheck(runs, model, group, threads, out_path);
    }
    else
    {
      status = search_run(runs[static_cast<std::size_t>(run_id)], run_id, model, group, threads, out_path);
    }
    rclcpp::shutdown();
    return status;
  }
  catch (const std::bad_alloc &)
  {
    std::cerr << "RESOURCE_LIMIT bad_alloc\n";
    return 3;
  }
  catch (const std::exception & error)
  {
    std::cerr << "EXECUTION_ERROR " << error.what() << "\n";
    return 4;
  }
}
