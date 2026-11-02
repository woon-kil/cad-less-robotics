// UR-ANALYTIC-IK-PROCESS-CONNECT-R3. run 6–11의 모든 원본 구간에 t=0.5를 넣는다.
// 기존 waypoint 후보와 이전 결과는 바꾸지 않는다.

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

#include <tesseract/kinematics/ur/ur_inv_kin.h>
#include <tesseract/kinematics/utils.h>

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
const double kModelPositionM = 1.0e-6;
const double kModelOrientationRad = 1.0e-6;
const double kPathPositionM = 0.001;
const double kPathOrientationRad = 0.01;
const double kLimitBoundaryRad = 1.0e-12;
const double kDuplicateAbsRad = 1.0e-12;
const double kTwoPiClassAbsRad = 1.0e-8;
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

struct Insert
{
  int from_sequence_id = -1;
  int to_sequence_id = -1;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  int solver_count = 0;
  int expanded_count = 0;
  int valid_count = 0;
  int in_limit_fk_violation_count = 0;
  int duplicate_count = 0;
  int out_of_limit_count = 0;
  int nonfinite_count = 0;
  double max_in_limit_position_m = std::numeric_limits<double>::quiet_NaN();
  double max_in_limit_orientation_rad = std::numeric_limits<double>::quiet_NaN();
  std::vector<Candidate> valid;
};

struct Rung
{
  bool inserted = false;
  int sequence_id = -1;
  int from_sequence_id = -1;
  int to_sequence_id = -1;
  double parameter_t = 0.0;
  std::vector<Candidate> candidates;
};

struct SampleStats
{
  double max_position = std::numeric_limits<double>::quiet_NaN();
  double max_orientation = std::numeric_limits<double>::quiet_NaN();
  int max_position_start = -1;
  int max_position_end = -1;
  double max_position_t = std::numeric_limits<double>::quiet_NaN();
  int max_orientation_start = -1;
  int max_orientation_end = -1;
  double max_orientation_t = std::numeric_limits<double>::quiet_NaN();
  bool within = true;
  int fk_count = 0;
};

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

bool within_limits(const Eigen::VectorXd & joints, const Eigen::MatrixX2d & limits)
{
  for (int index = 0; index < joints.size(); ++index)
  {
    if (!std::isfinite(joints[index]) || joints[index] < limits(index, 0) - kLimitBoundaryRad ||
        joints[index] > limits(index, 1) + kLimitBoundaryRad)
    {
      return false;
    }
  }
  return true;
}

bool exact_same_joints(const Eigen::VectorXd & left, const Eigen::VectorXd & right)
{
  for (int index = 0; index < left.size(); ++index)
  {
    if (std::abs(left[index] - right[index]) > kDuplicateAbsRad)
    {
      return false;
    }
  }
  return true;
}

bool same_two_pi_class(const Eigen::VectorXd & left, const Eigen::VectorXd & right)
{
  for (int index = 0; index < left.size(); ++index)
  {
    const double delta = left[index] - right[index];
    const double turns = std::round(delta / (2.0 * M_PI));
    if (std::abs(delta - turns * 2.0 * M_PI) > kTwoPiClassAbsRad)
    {
      return false;
    }
  }
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

class SubsegmentEdgeEvaluator : public descartes_light::EdgeEvaluator<double>
{
public:
  SubsegmentEdgeEvaluator(
    Eigen::Vector3d start_position,
    Eigen::Vector3d end_position,
    Eigen::Quaterniond start_orientation,
    Eigen::Quaterniond end_orientation,
    double t_start,
    double t_end,
    moveit::core::RobotModelConstPtr model,
    const moveit::core::JointModelGroup * group)
  : start_position_(std::move(start_position)),
    end_position_(std::move(end_position)),
    start_orientation_(std::move(start_orientation)),
    end_orientation_(std::move(end_orientation)),
    t_start_(t_start),
    t_end_(t_end),
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
    const bool accepted = accept(start.values, end.values, &checked, &nonfinite);
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

  std::uint64_t fk_checks() const { return fk_checks_.load(std::memory_order_relaxed); }
  std::uint64_t early_rejects() const { return early_rejects_.load(std::memory_order_relaxed); }
  std::uint64_t nonfinite_rejects() const { return nonfinite_rejects_.load(std::memory_order_relaxed); }

private:
  bool accept(
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
    for (int sample_index : sample_order())
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
      if (!(position_error <= kPathPositionM) || !(orientation_error <= kPathOrientationRad))
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
    const double local = static_cast<double>(sample_index) / static_cast<double>(kSampleDenominator);
    const double parameter = (1.0 - local) * t_start_ + local * t_end_;
    const Eigen::VectorXd joints = (1.0 - local) * start_joints + local * end_joints;
    if (!std::isfinite(parameter) || !joints.allFinite())
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
    const Eigen::Vector3d target_position =
      (1.0 - parameter) * start_position_ + parameter * end_position_;
    const Eigen::Quaterniond target_orientation = start_orientation_.slerp(parameter, end_orientation_);
    const Eigen::Matrix3d target_matrix = target_orientation.toRotationMatrix();
    if (!target_position.allFinite() || !target_matrix.allFinite() || !actual.translation().allFinite() ||
        !actual.rotation().allFinite())
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
  const double t_start_;
  const double t_end_;
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

  std::uint64_t edge_evaluations() const { return edge_evaluations_.load(std::memory_order_relaxed); }

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
  if (read_pod<std::uint32_t>(input) != static_cast<std::uint32_t>(kRunCount))
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
    auto & run = runs[static_cast<std::size_t>(run_id)];
    run.resize(waypoint_count);
    for (std::uint32_t waypoint_index = 0; waypoint_index < waypoint_count; ++waypoint_index)
    {
      Waypoint & waypoint = run[waypoint_index];
      waypoint.sequence_id = read_pod<std::int32_t>(input);
      double position[3];
      double orientation[4];
      input.read(reinterpret_cast<char *>(position), sizeof(position));
      input.read(reinterpret_cast<char *>(orientation), sizeof(orientation));
      waypoint.position = Eigen::Vector3d(position[0], position[1], position[2]);
      waypoint.orientation = Eigen::Quaterniond(orientation[3], orientation[0], orientation[1], orientation[2]);
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
        if (!input || (candidate.source_code != 0 && candidate.source_code != 1))
        {
          return false;
        }
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

Insert solve_insert(
  const Waypoint & start,
  const Waypoint & end,
  tesseract::kinematics::URInvKin & solver,
  const std::string & tip_link,
  const Eigen::MatrixX2d & limits,
  const std::vector<Eigen::Index> & redundant_joints,
  moveit::core::RobotState & state,
  const moveit::core::JointModelGroup * group)
{
  Insert insert;
  insert.from_sequence_id = start.sequence_id;
  insert.to_sequence_id = end.sequence_id;
  insert.position = 0.5 * start.position + 0.5 * end.position;
  const Eigen::Quaterniond interpolated = start.orientation.slerp(0.5, end.orientation);
  const bool usable = interpolated.coeffs().allFinite() && interpolated.norm() > 0.0;
  insert.orientation = usable ? interpolated.normalized() : Eigen::Quaterniond::Identity();
  if (!insert.position.allFinite() || !usable)
  {
    insert.nonfinite_count = 1;
    return insert;
  }
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.linear() = insert.orientation.toRotationMatrix();
  target.translation() = insert.position;
  tesseract::kinematics::IKSolutions solved;
  tesseract::common::LinkIdTransformMap tip_poses;
  tip_poses.emplace(tip_link, target);
  const Eigen::VectorXd seed = Eigen::VectorXd::Zero(6);
  solver.calcInvKin(solved, tip_poses, seed);
  insert.solver_count = static_cast<int>(solved.size());
  std::vector<Eigen::VectorXd> finite_joints;
  std::vector<int> finite_classes;
  int class_count = 0;
  auto consider = [&](const Eigen::VectorXd & joints, int source_code) {
    ++insert.expanded_count;
    if (!joints.allFinite())
    {
      ++insert.nonfinite_count;
      return;
    }
    for (const auto & earlier : finite_joints)
    {
      if (exact_same_joints(joints, earlier))
      {
        ++insert.duplicate_count;
        finite_joints.push_back(joints);
        finite_classes.push_back(-1);
        return;
      }
    }
    int class_id = -1;
    for (std::size_t class_index = 0; class_index < finite_joints.size(); ++class_index)
    {
      if (finite_classes[class_index] >= 0 && same_two_pi_class(joints, finite_joints[class_index]))
      {
        class_id = finite_classes[class_index];
        break;
      }
    }
    if (class_id < 0)
    {
      class_id = class_count;
      ++class_count;
    }
    finite_joints.push_back(joints);
    finite_classes.push_back(class_id);
    if (!within_limits(joints, limits))
    {
      ++insert.out_of_limit_count;
      return;
    }
    std::vector<double> query(joints.data(), joints.data() + joints.size());
    state.setJointGroupPositions(group, query);
    state.update();
    const Eigen::Isometry3d actual = tool_in_base(state);
    const double position_error = (actual.translation() - target.translation()).norm();
    const double orientation_error = rotation_error_rad(target.rotation(), actual.rotation());
    if (!std::isfinite(position_error) || !std::isfinite(orientation_error))
    {
      ++insert.nonfinite_count;
      return;
    }
    if (!std::isfinite(insert.max_in_limit_position_m) || position_error > insert.max_in_limit_position_m)
    {
      insert.max_in_limit_position_m = position_error;
    }
    if (!std::isfinite(insert.max_in_limit_orientation_rad) ||
        orientation_error > insert.max_in_limit_orientation_rad)
    {
      insert.max_in_limit_orientation_rad = orientation_error;
    }
    const bool model_pass = position_error <= kModelPositionM && orientation_error <= kModelOrientationRad;
    if (!model_pass)
    {
      ++insert.in_limit_fk_violation_count;
      return;
    }
    Candidate candidate;
    candidate.source_code = source_code;
    candidate.kinematic_class_id = class_id;
    candidate.joints = joints;
    insert.valid.push_back(std::move(candidate));
    ++insert.valid_count;
  };
  for (const auto & solution : solved)
  {
    consider(solution, 0);
    if (!solution.allFinite())
    {
      continue;
    }
    const auto expanded = tesseract::kinematics::getRedundantSolutions<double>(solution, limits, redundant_joints);
    for (const auto & extra : expanded)
    {
      consider(extra, 1);
    }
  }
  return insert;
}

void write_candidate(std::ostream & output, const Candidate & candidate, std::size_t index)
{
  output << "{\"candidate_index\": " << index
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

bool verify_original(
  const std::vector<Waypoint> & waypoints,
  const std::vector<Insert> & inserts,
  const std::vector<std::size_t> & chosen,
  moveit::core::RobotState & state,
  const moveit::core::JointModelGroup * group,
  SampleStats * stats)
{
  stats->within = true;
  for (std::size_t segment = 0; segment < inserts.size(); ++segment)
  {
    const Waypoint & start = waypoints[segment];
    const Waypoint & end = waypoints[segment + 1];
    const Insert & insert = inserts[segment];
    const std::size_t start_index = chosen[segment * 2];
    const std::size_t insert_index = chosen[segment * 2 + 1];
    const std::size_t end_index = chosen[segment * 2 + 2];
    const Eigen::VectorXd & start_joints = start.candidates[start_index].joints;
    const Eigen::VectorXd & mid_joints = insert.valid[insert_index].joints;
    const Eigen::VectorXd & end_joints = end.candidates[end_index].joints;
    for (int sample_index = 0; sample_index <= kSampleDenominator; ++sample_index)
    {
      const double parameter = static_cast<double>(sample_index) / static_cast<double>(kSampleDenominator);
      Eigen::VectorXd joints;
      if (parameter <= 0.5)
      {
        const double local = parameter / 0.5;
        joints = (1.0 - local) * start_joints + local * mid_joints;
      }
      else
      {
        const double local = (parameter - 0.5) / 0.5;
        joints = (1.0 - local) * mid_joints + local * end_joints;
      }
      ++stats->fk_count;
      if (!joints.allFinite())
      {
        stats->within = false;
        continue;
      }
      std::vector<double> query(joints.data(), joints.data() + joints.size());
      state.setJointGroupPositions(group, query);
      state.update();
      const Eigen::Isometry3d actual = tool_in_base(state);
      const Eigen::Vector3d target_position = (1.0 - parameter) * start.position + parameter * end.position;
      const Eigen::Quaterniond target_orientation = start.orientation.slerp(parameter, end.orientation);
      const Eigen::Matrix3d target_matrix = target_orientation.toRotationMatrix();
      if (!target_position.allFinite() || !target_matrix.allFinite() || !actual.translation().allFinite() ||
          !actual.rotation().allFinite())
      {
        stats->within = false;
        continue;
      }
      const double position_error = (actual.translation() - target_position).norm();
      const double orientation_error = rotation_error_rad(target_matrix, actual.rotation());
      if (!std::isfinite(position_error) || !std::isfinite(orientation_error) ||
          !(position_error <= kPathPositionM) || !(orientation_error <= kPathOrientationRad))
      {
        stats->within = false;
      }
      if (std::isfinite(position_error) &&
          (!std::isfinite(stats->max_position) || position_error > stats->max_position))
      {
        stats->max_position = position_error;
        stats->max_position_start = start.sequence_id;
        stats->max_position_end = end.sequence_id;
        stats->max_position_t = parameter;
      }
      if (std::isfinite(orientation_error) &&
          (!std::isfinite(stats->max_orientation) || orientation_error > stats->max_orientation))
      {
        stats->max_orientation = orientation_error;
        stats->max_orientation_start = start.sequence_id;
        stats->max_orientation_end = end.sequence_id;
        stats->max_orientation_t = parameter;
      }
    }
  }
  return stats->within;
}

std::shared_ptr<FiniteCostGate> make_gate(
  const Waypoint & start,
  const Waypoint & end,
  double t_start,
  double t_end,
  const moveit::core::RobotModelConstPtr & model,
  const moveit::core::JointModelGroup * group,
  const descartes_light::EdgeEvaluator<double>::ConstPtr & distance,
  std::shared_ptr<SubsegmentEdgeEvaluator> * accuracy_out)
{
  auto accuracy = std::make_shared<SubsegmentEdgeEvaluator>(
    start.position, end.position, start.orientation, end.orientation, t_start, t_end, model, group);
  *accuracy_out = accuracy;
  return std::make_shared<FiniteCostGate>(accuracy, distance);
}

void write_inserts(std::ostream & output, const std::vector<Insert> & inserts)
{
  output << "  \"insert_count\": " << inserts.size() << ",\n  \"inserts\": [\n";
  for (std::size_t index = 0; index < inserts.size(); ++index)
  {
    if (index)
    {
      output << ",\n";
    }
    const Insert & insert = inserts[index];
    output << "    {\"from_sequence_id\": " << insert.from_sequence_id
           << ", \"to_sequence_id\": " << insert.to_sequence_id
           << ", \"t\": 0.5, \"position_m\": [";
    write_number(output, insert.position.x());
    output << ", ";
    write_number(output, insert.position.y());
    output << ", ";
    write_number(output, insert.position.z());
    output << "], \"orientation_quaternion_normalized_xyzw\": [";
    write_number(output, insert.orientation.x());
    output << ", ";
    write_number(output, insert.orientation.y());
    output << ", ";
    write_number(output, insert.orientation.z());
    output << ", ";
    write_number(output, insert.orientation.w());
    output << "], \"solver_solution_count\": " << insert.solver_count
           << ", \"expanded_candidate_count\": " << insert.expanded_count
           << ", \"valid_count\": " << insert.valid_count
           << ", \"in_limit_fk_violation_count\": " << insert.in_limit_fk_violation_count
           << ", \"duplicate_count\": " << insert.duplicate_count
           << ", \"out_of_limit_count\": " << insert.out_of_limit_count
           << ", \"nonfinite_count\": " << insert.nonfinite_count
           << ", \"max_in_limit_position_m\": ";
    write_number(output, insert.max_in_limit_position_m);
    output << ", \"max_in_limit_orientation_rad\": ";
    write_number(output, insert.max_in_limit_orientation_rad);
    output << ", \"valid_candidates\": [";
    for (std::size_t candidate_index = 0; candidate_index < insert.valid.size(); ++candidate_index)
    {
      if (candidate_index)
      {
        output << ", ";
      }
      write_candidate(output, insert.valid[candidate_index], candidate_index);
    }
    output << "]}";
  }
  output << "\n  ],\n";
}

int precheck(
  const std::vector<Waypoint> & waypoints,
  tesseract::kinematics::URInvKin & solver,
  const std::string & tip_link,
  const Eigen::MatrixX2d & limits,
  const std::vector<Eigen::Index> & redundant_joints,
  moveit::core::RobotState & state,
  const moveit::core::RobotModelConstPtr & model,
  const moveit::core::JointModelGroup * group,
  int threads,
  const std::string & out_path)
{
  const Insert insert = solve_insert(
    waypoints[0], waypoints[1], solver, tip_link, limits, redundant_joints, state, group);
  auto distance = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<double>>();
  std::shared_ptr<SubsegmentEdgeEvaluator> accuracy;
  auto gate = make_gate(waypoints[0], waypoints[1], 0.0, 0.5, model, group, distance, &accuracy);
  std::vector<std::pair<bool, double>> single;
  std::vector<std::pair<bool, double>> parallel;
  std::vector<bool> present;
  for (const auto & pair : kPrecheckPairs)
  {
    const bool from_ok = pair[0] >= 0 && pair[0] < kCandidateCount;
    const bool to_ok = pair[1] >= 0 && pair[1] < static_cast<int>(insert.valid.size());
    present.push_back(from_ok && to_ok);
    if (!present.back())
    {
      single.emplace_back(false, 0.0);
      parallel.emplace_back(false, 0.0);
      continue;
    }
    descartes_light::State<double> start(waypoints[0].candidates[static_cast<std::size_t>(pair[0])].joints);
    descartes_light::State<double> finish(insert.valid[static_cast<std::size_t>(pair[1])].joints);
    single.push_back(gate->evaluate(start, finish));
  }
  parallel.resize(single.size());
#pragma omp parallel for num_threads(threads) schedule(static)
  for (int index = 0; index < 10; ++index)
  {
    if (!present[static_cast<std::size_t>(index)])
    {
      parallel[static_cast<std::size_t>(index)] = std::make_pair(false, 0.0);
      continue;
    }
    descartes_light::State<double> start(
      waypoints[0].candidates[static_cast<std::size_t>(kPrecheckPairs[index][0])].joints);
    descartes_light::State<double> finish(
      insert.valid[static_cast<std::size_t>(kPrecheckPairs[index][1])].joints);
    parallel[static_cast<std::size_t>(index)] = gate->evaluate(start, finish);
  }
  int valid_count = 0;
  int invalid_count = 0;
  int absent_count = 0;
  bool matched = true;
  for (std::size_t index = 0; index < present.size(); ++index)
  {
    if (!present[index])
    {
      ++absent_count;
      continue;
    }
    if (single[index].first)
    {
      ++valid_count;
    }
    else
    {
      ++invalid_count;
    }
    if (single[index].first != parallel[index].first ||
        !(std::isfinite(single[index].second) && single[index].second == parallel[index].second))
    {
      matched = false;
    }
  }
  const char * scope = "BOTH";
  if (valid_count == 0 && invalid_count == 0)
  {
    scope = "EMPTY";
  }
  else if (valid_count == 0)
  {
    scope = "ONLY_INVALID";
  }
  else if (invalid_count == 0)
  {
    scope = "ONLY_VALID";
  }
  std::ofstream output(out_path);
  output << std::setprecision(17);
  output << "{\n  \"gate\": \"UR-ANALYTIC-IK-PROCESS-CONNECT-R3\",\n"
         << "  \"role\": \"THREAD_PRECHECK\",\n"
         << "  \"precheck_edge\": \"run 6 first sub-edge, sequences 211-212, t 0 to 0.5\",\n"
         << "  \"threads\": " << threads
         << ",\n  \"precheck_match\": " << (matched ? "true" : "false")
         << ",\n  \"valid_edge_count\": " << valid_count
         << ",\n  \"invalid_edge_count\": " << invalid_count
         << ",\n  \"absent_edge_count\": " << absent_count
         << ",\n  \"precheck_scope\": \"" << scope << "\",\n  \"edges\": [\n";
  for (std::size_t index = 0; index < present.size(); ++index)
  {
    if (index)
    {
      output << ",\n";
    }
    output << "    {\"from\": " << kPrecheckPairs[index][0] << ", \"to\": " << kPrecheckPairs[index][1]
           << ", \"present\": " << (present[index] ? "true" : "false")
           << ", \"single_valid\": " << (present[index] && single[index].first ? "true" : "false")
           << ", \"multi_valid\": " << (present[index] && parallel[index].first ? "true" : "false")
           << ", \"single_cost\": ";
    write_number(output, present[index] ? single[index].second : std::numeric_limits<double>::quiet_NaN());
    output << ", \"multi_cost\": ";
    write_number(output, present[index] ? parallel[index].second : std::numeric_limits<double>::quiet_NaN());
    output << "}";
  }
  output << "\n  ],\n  \"complete\": true\n}\n";
  std::cout << "PRECHECK " << (matched ? "MATCH" : "MISMATCH") << " " << scope << "\n";
  return matched ? 0 : 5;
}

int search_run(
  const std::vector<Waypoint> & waypoints,
  int run_id,
  tesseract::kinematics::URInvKin & solver,
  const std::string & tip_link,
  const Eigen::MatrixX2d & limits,
  const std::vector<Eigen::Index> & redundant_joints,
  moveit::core::RobotState & ik_state,
  const moveit::core::RobotModelConstPtr & model,
  const moveit::core::JointModelGroup * group,
  int threads,
  const std::string & out_path)
{
  const auto started = std::chrono::steady_clock::now();
  std::vector<Insert> inserts;
  inserts.reserve(waypoints.size() - 1);
  bool missing_candidate = false;
  for (std::size_t segment = 0; segment + 1 < waypoints.size(); ++segment)
  {
    inserts.push_back(solve_insert(
      waypoints[segment], waypoints[segment + 1], solver, tip_link, limits, redundant_joints, ik_state, group));
    if (inserts.back().valid.empty())
    {
      missing_candidate = true;
    }
  }
  if (static_cast<int>(inserts.size()) != static_cast<int>(waypoints.size()) - 1)
  {
    std::cerr << "EXECUTION_ERROR insert_count\n";
    return 4;
  }
  std::string search_status = missing_candidate ? "NO_INSERT_CANDIDATE" : "NO_PATH";
  std::string accuracy_status = "NOT_EVALUATED";
  double ladder_cost = std::numeric_limits<double>::quiet_NaN();
  std::vector<Rung> rungs;
  std::vector<std::size_t> chosen;
  std::uint64_t edge_count = 0;
  std::uint64_t early_count = 0;
  std::uint64_t fk_count = 0;
  std::uint64_t nonfinite_count = 0;
  SampleStats stats;
  if (!missing_candidate)
  {
    for (std::size_t segment = 0; segment < inserts.size(); ++segment)
    {
      Rung start_rung;
      start_rung.sequence_id = waypoints[segment].sequence_id;
      start_rung.parameter_t = 0.0;
      start_rung.candidates = waypoints[segment].candidates;
      rungs.push_back(std::move(start_rung));
      Rung mid_rung;
      mid_rung.inserted = true;
      mid_rung.from_sequence_id = inserts[segment].from_sequence_id;
      mid_rung.to_sequence_id = inserts[segment].to_sequence_id;
      mid_rung.parameter_t = 0.5;
      mid_rung.candidates = inserts[segment].valid;
      rungs.push_back(std::move(mid_rung));
    }
    Rung last;
    last.sequence_id = waypoints.back().sequence_id;
    last.parameter_t = 1.0;
    last.candidates = waypoints.back().candidates;
    rungs.push_back(std::move(last));
    std::vector<std::vector<descartes_light::State<double>::ConstPtr>> states(rungs.size());
    std::vector<descartes_light::WaypointSampler<double>::ConstPtr> samplers;
    std::vector<descartes_light::EdgeEvaluator<double>::ConstPtr> gates;
    std::vector<std::shared_ptr<FiniteCostGate>> gate_owners;
    std::vector<std::shared_ptr<SubsegmentEdgeEvaluator>> accuracies;
    auto distance = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<double>>();
    for (std::size_t rung_index = 0; rung_index < rungs.size(); ++rung_index)
    {
      for (const Candidate & candidate : rungs[rung_index].candidates)
      {
        states[rung_index].push_back(std::make_shared<descartes_light::State<double>>(candidate.joints));
      }
      samplers.push_back(std::make_shared<RecordedJointSampler>(states[rung_index]));
    }
    for (std::size_t segment = 0; segment < inserts.size(); ++segment)
    {
      for (int half = 0; half < 2; ++half)
      {
        std::shared_ptr<SubsegmentEdgeEvaluator> accuracy;
        auto gate = make_gate(
          waypoints[segment], waypoints[segment + 1], half == 0 ? 0.0 : 0.5, half == 0 ? 0.5 : 1.0,
          model, group, distance, &accuracy);
        accuracies.push_back(accuracy);
        gate_owners.push_back(gate);
        gates.push_back(gate);
      }
    }
    descartes_light::LadderGraphSolver<double> ladder(threads);
    const descartes_light::BuildStatus built = ladder.build(samplers, gates, {});
    bool have_path = false;
    if (built)
    {
      try
      {
        const descartes_light::SearchResult<double> result = ladder.search();
        if (result.trajectory.size() != rungs.size() || !std::isfinite(result.cost))
        {
          std::cerr << "EXECUTION_ERROR path\n";
          return 4;
        }
        chosen.resize(rungs.size());
        for (std::size_t rung_index = 0; rung_index < rungs.size(); ++rung_index)
        {
          const auto match = std::find(
            states[rung_index].begin(), states[rung_index].end(), result.trajectory[rung_index]);
          if (match == states[rung_index].end() ||
              rungs[rung_index].candidates[static_cast<std::size_t>(match - states[rung_index].begin())].joints !=
              result.trajectory[rung_index]->values)
          {
            std::cerr << "EXECUTION_ERROR identity\n";
            return 4;
          }
          chosen[rung_index] = static_cast<std::size_t>(match - states[rung_index].begin());
        }
        ladder_cost = result.cost;
        have_path = true;
      }
      catch (const std::runtime_error &)
      {
        have_path = false;
      }
    }
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
    if (have_path)
    {
      moveit::core::RobotState verification_state(model);
      const bool verified = verify_original(
        waypoints, inserts, chosen, verification_state, group, &stats);
      if (!verified)
      {
        search_status = "CHECK_MISMATCH";
      }
      else
      {
        search_status = "FOUND";
        accuracy_status = "SAMPLED_ACCURACY_PASS";
      }
    }
  }
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  std::ofstream output(out_path);
  output << std::setprecision(17);
  output << "{\n  \"gate\": \"UR-ANALYTIC-IK-PROCESS-CONNECT-R3\",\n"
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
  output << ",\n  \"max_position_sequence_ids\": [" << stats.max_position_start << ", "
         << stats.max_position_end << "],\n  \"max_position_t\": ";
  write_number(output, stats.max_position_t);
  output << ",\n  \"max_orientation_error_rad\": ";
  write_number(output, stats.max_orientation);
  output << ",\n  \"max_orientation_sequence_ids\": [" << stats.max_orientation_start << ", "
         << stats.max_orientation_end << "],\n  \"max_orientation_t\": ";
  write_number(output, stats.max_orientation_t);
  output << ",\n";
  write_inserts(output, inserts);
  output << "  \"selected\": [";
  if (!chosen.empty())
  {
    for (std::size_t rung_index = 0; rung_index < rungs.size(); ++rung_index)
    {
      if (rung_index)
      {
        output << ", ";
      }
      const Rung & rung = rungs[rung_index];
      const Candidate & candidate = rung.candidates[chosen[rung_index]];
      output << "{\"kind\": \"" << (rung.inserted ? "INSERT" : "WAYPOINT") << "\"";
      if (rung.inserted)
      {
        output << ", \"from_sequence_id\": " << rung.from_sequence_id
               << ", \"to_sequence_id\": " << rung.to_sequence_id << ", \"t\": 0.5";
      }
      else
      {
        output << ", \"sequence_id\": " << rung.sequence_id;
      }
      output << ", \"candidate_index\": " << chosen[rung_index]
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
            << " inserts " << inserts.size() << " edges " << edge_count
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
      std::cerr << "usage: ur_analytic_ik_process_connect_r3 precheck|run ...\n";
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
        std::cerr << "usage: ur_analytic_ik_process_connect_r3 run ID urdf srdf runs.bin out.json\n";
        return 2;
      }
      run_id = std::stoi(argv[2]);
      urdf_path = argv[3];
      srdf_path = argv[4];
      runs_path = argv[5];
      out_path = argv[6];
      if (run_id < 6 || run_id > 11)
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
    if (group == nullptr)
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
    Eigen::MatrixX2d limits(6, 2);
    std::vector<tesseract::common::JointId> joint_ids;
    for (int index = 0; index < 6; ++index)
    {
      if (joint_names[static_cast<std::size_t>(index)] != kJointNames[index])
      {
        std::cerr << "EXECUTION_ERROR joints\n";
        return 4;
      }
      const auto & bounds = group->getActiveJointModels()[static_cast<std::size_t>(index)]->getVariableBounds();
      if (bounds.size() != 1 || !bounds[0].position_bounded_)
      {
        std::cerr << "EXECUTION_ERROR limits\n";
        return 4;
      }
      limits(index, 0) = bounds[0].min_position_;
      limits(index, 1) = bounds[0].max_position_;
      joint_ids.emplace_back(kJointNames[index]);
    }
    const std::string tip_link = "tool0";
    tesseract::kinematics::URInvKin solver(
      tesseract::kinematics::UR5eParameters, tesseract::common::LinkId("base_link"),
      tesseract::common::LinkId(tip_link), joint_ids);
    const std::vector<Eigen::Index> redundant_joints{0, 1, 2, 3, 4, 5};
    moveit::core::RobotState ik_state(model);
    const int threads = std::max(1, static_cast<int>(std::thread::hardware_concurrency()));
    const int status = mode == "precheck" ?
      precheck(runs[6], solver, tip_link, limits, redundant_joints, ik_state, model, group, threads, out_path) :
      search_run(
        runs[static_cast<std::size_t>(run_id)], run_id, solver, tip_link, limits, redundant_joints,
        ik_state, model, group, threads, out_path);
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
