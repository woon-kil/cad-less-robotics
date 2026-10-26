// Copyright 2026 cad-less-robotics contributors
// P-005 attempt 5. projected joint waypoint를 관절 공간에서 직접 잇는다.
// q를 FK한 뒤 IK로 다시 풀지 않는다. Cartesian planner 파라미터는 쓰지 않는다.

#include "robot_controller/capability_adapter.hpp"
#include "robot_controller/motion_contract.hpp"
#include "vision_path_generator/polyline.hpp"

#include <moveit/collision_detection/collision_common.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <urdf/model.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <sys/wait.h>
#include <vector>

namespace
{
constexpr char kSnapshotSha[] = "09fa059c0ae4fbda4a1b90dcc4c769936562ad1040aa8824b370aed8e1a16a60";
constexpr char kWaypointSha[] = "b9a254590dcc260b4556dd38c366bdd98a64efe99bd5eee4d614b39834041021";
constexpr double kEefStep = 0.002;
constexpr double kJumpThreshold = 1.0;
constexpr double kPositionMargin = 1e-6;
constexpr int kExecuteCalls = 0;
constexpr int kControllerSends = 0;
constexpr int kTrajectoryIkCalls = 0;

int g_projection_ik_calls = 0;

struct FrozenCase
{
  const char* id;
  std::size_t start;
  std::size_t end;
  double translation[3];
  double rotation[9];
};

constexpr FrozenCase kFrozenCases[] = {
    {"CASE-01", 61, 71,
     {-0.42602454754951585, 0.19205943289939401, -0.69840859940510502},
     {0.204375830042828, 0.8963690680027564, 0.39338659613943999, -0.84085154582838639, -0.045003468936162694,
      0.53939166258084192, 0.50119776333531074, -0.44101834623015579, 0.74451569514423288}},
    {"CASE-02", 312, 322,
     {-0.29751000908758757, 0.46901838554202546, -0.51596554967778152},
     {-0.61089138166335921, 0.069461751697390384, 0.78866138796099383, -0.25905016123996089, -0.95884767261346937,
      -0.11620737792964075, 0.74813416829535417, -0.2752929453779413, 0.60374585750475851}},
    {"CASE-03", 572, 582,
     {-0.43689984528870657, -0.24653509849787827, -0.48107733843494255},
     {0.15896260049157818, -0.83701046661437173, 0.52358797772957555, 0.98607318826525658, 0.16086523927024865,
      -0.042214241422974585, -0.048893343405277984, 0.52300655213153613, 0.85092513618892252}},
};

struct Scene
{
  moveit::core::RobotModelConstPtr model;
  const moveit::core::JointModelGroup* arm{nullptr};
  const moveit::core::JointModelGroup* grip{nullptr};
  const moveit::core::LinkModel* end_link{nullptr};
  std::string grip_state;
  std::vector<std::string> joint_names;
  planning_scene_monitor::PlanningSceneMonitorPtr monitor;
  const kinematics::KinematicsBase* solver{nullptr};
  int linear_row{0};
};

struct WaypointCapability
{
  std::size_t source_index{0};
  robot_controller::MotionPose desired;
  robot_controller::CapabilitySelection selection;
  robot_controller::JointStep step;
  robot_controller::OrientationMetrics desired_change;
};

struct StoredProjection
{
  std::size_t source_index{0};
  std::string capability_class;
  std::vector<double> joints;
  double position_error_m{0.0};
  double tool_axis_error_deg{0.0};
  double roll_error_deg{0.0};
  double full_orientation_error_deg{0.0};
};

struct Attempt4Row
{
  double fraction{0.0};
  std::string planning;
};

struct SampleRow
{
  int sample_index{0};
  double time_from_start_sec{0.0};
  std::size_t source_index{std::numeric_limits<std::size_t>::max()};
  std::vector<double> joints;
  bool within_limits{false};
  bool self_collision{false};
  bool full_collision{false};
};

struct FkRow
{
  std::size_t source_index{0};
  robot_controller::OrientationMetrics versus_projected;
  robot_controller::OrientationMetrics versus_desired;
};

struct SegmentRow
{
  std::size_t from_source{0};
  std::size_t to_source{0};
  robot_controller::JointStep step;
  robot_controller::CartesianSegmentDeviation cartesian;
  int intervals{0};
};

struct JointCase
{
  std::string trajectory_class;
  std::string process_fidelity;
  int waypoint_count{0};
  int interpolated_count{0};
  int limit_violations{0};
  int self_collision{0};
  int full_collision{0};
  int branch_discontinuities{0};
  bool timing_ok{false};
  bool velocity_ok{false};
  bool trajectory_built{false};
  bool fk_waypoint_consistent{false};
  double max_velocity{0.0};
  double max_acceleration{0.0};
  double duration_sec{0.0};
  double max_fk_projected_position_m{0.0};
  double max_fk_projected_orientation_deg{0.0};
  double max_chord_deviation_m{0.0};
  double max_tool_axis_excursion_deg{0.0};
  double max_cartesian_path_length_m{0.0};
  robot_controller::ErrorSummary joint_step;
  robot_controller::ErrorSummary position;
  robot_controller::ErrorSummary tool;
  robot_controller::ErrorSummary roll;
  robot_controller::ErrorSummary full;
  std::vector<robot_controller::SealedProjection> sealed;
  std::vector<SampleRow> samples;
  std::vector<FkRow> fk_rows;
  std::vector<SegmentRow> segments;
};

class JointStatePump
{
public:
  JointStatePump(const rclcpp::Node::SharedPtr& node, const std::vector<std::string>& names)
  : names_(names)
  {
    publisher_ = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());
    timer_ = node->create_wall_timer(std::chrono::milliseconds(20), [this]() { publish(); });
  }

  void publish()
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = rclcpp::Clock().now();
    message.name = names_;
    message.position.assign(names_.size(), 0.0);
    publisher_->publish(message);
  }

private:
  std::vector<std::string> names_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

std::string readText(const std::string& path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("파일을 열 수 없습니다: " + path);
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

std::string sha256File(const std::string& path)
{
  int fds[2];
  if (pipe(fds) != 0) {
    throw std::runtime_error("sha256 파이프를 만들지 못했습니다.");
  }
  const pid_t pid = fork();
  if (pid < 0) {
    throw std::runtime_error("sha256 프로세스를 만들지 못했습니다.");
  }
  if (pid == 0) {
    dup2(fds[1], STDOUT_FILENO);
    close(fds[0]);
    close(fds[1]);
    execlp("sha256sum", "sha256sum", path.c_str(), static_cast<char*>(nullptr));
    _exit(127);
  }
  close(fds[1]);
  std::string output;
  char buffer[256];
  ssize_t count = 0;
  while ((count = read(fds[0], buffer, sizeof(buffer))) > 0) {
    output.append(buffer, static_cast<std::size_t>(count));
  }
  close(fds[0]);
  int status = 0;
  waitpid(pid, &status, 0);
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0 || output.size() < 64) {
    throw std::runtime_error("sha256sum 실패: " + path);
  }
  return output.substr(0, 64);
}

std::vector<std::string> splitCsv(const std::string& line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

std::vector<robot_controller::MotionPose> loadWaypoints(const std::string& path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("waypoint CSV를 열 수 없습니다.");
  }
  std::string header;
  std::getline(input, header);
  const auto columns = splitCsv(header);
  auto indexOf = [&](const std::string& name) {
    const auto found = std::find(columns.begin(), columns.end(), name);
    if (found == columns.end()) {
      throw std::runtime_error("waypoint CSV에 열이 없습니다: " + name);
    }
    return static_cast<std::size_t>(std::distance(columns.begin(), found));
  };
  const std::size_t i_index = indexOf("index");
  const std::size_t i_x = indexOf("x");
  const std::size_t i_y = indexOf("y");
  const std::size_t i_z = indexOf("z");
  const std::size_t i_qx = indexOf("qx");
  const std::size_t i_qy = indexOf("qy");
  const std::size_t i_qz = indexOf("qz");
  const std::size_t i_qw = indexOf("qw");
  const std::size_t i_nx = indexOf("nx");
  const std::size_t i_ny = indexOf("ny");
  const std::size_t i_nz = indexOf("nz");
  std::vector<robot_controller::MotionPose> poses;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }
    const auto fields = splitCsv(line);
    robot_controller::MotionPose pose;
    pose.source_index = static_cast<std::size_t>(std::stoul(fields.at(i_index)));
    pose.x = std::stod(fields.at(i_x));
    pose.y = std::stod(fields.at(i_y));
    pose.z = std::stod(fields.at(i_z));
    pose.qx = std::stod(fields.at(i_qx));
    pose.qy = std::stod(fields.at(i_qy));
    pose.qz = std::stod(fields.at(i_qz));
    pose.qw = std::stod(fields.at(i_qw));
    pose.nx = std::stod(fields.at(i_nx));
    pose.ny = std::stod(fields.at(i_ny));
    pose.nz = std::stod(fields.at(i_nz));
    poses.push_back(pose);
  }
  return poses;
}

vision_path_generator::Vec3 pointOf(const robot_controller::MotionPose& pose)
{
  return {pose.x, pose.y, pose.z};
}

vision_path_generator::Vec3 normalOf(const robot_controller::MotionPose& pose)
{
  return {pose.nx, pose.ny, pose.nz};
}

std::vector<robot_controller::PairAssessment> assessPairs(
    const std::vector<robot_controller::MotionPose>& poses, double max_gap)
{
  std::vector<robot_controller::PairAssessment> pairs;
  for (std::size_t index = 0; index + 1 < poses.size(); ++index) {
    const bool has_previous = index > 0;
    const bool has_following = index + 2 < poses.size();
    const auto kind = vision_path_generator::classifyNormalDiscontinuity(
        has_previous ? normalOf(poses[index - 1]) : vision_path_generator::Vec3{}, normalOf(poses[index]),
        normalOf(poses[index + 1]),
        has_following ? normalOf(poses[index + 2]) : vision_path_generator::Vec3{}, pointOf(poses[index]),
        pointOf(poses[index + 1]), robot_controller::poseDistance(poses[index], poses[index + 1]), max_gap,
        has_previous, has_following);
    robot_controller::PairAssessment pair;
    pair.index = index;
    pair.tool_z_deg = robot_controller::toolAxisAngleDeg(poses[index], poses[index + 1]);
    pair.quaternion_deg = robot_controller::quaternionStepDeg(poses[index], poses[index + 1]);
    if (kind == vision_path_generator::NormalDiscontinuity::Geometric) {
      pair.label = robot_controller::PairLabel::ExplainedGeometric;
    } else if (kind == vision_path_generator::NormalDiscontinuity::None) {
      pair.label = robot_controller::PairLabel::Continuous;
    } else {
      pair.label = robot_controller::PairLabel::Invalid;
    }
    pairs.push_back(pair);
  }
  return pairs;
}

robot_controller::MotionPose poseFromState(const moveit::core::RobotState& state, const std::string& link)
{
  const Eigen::Isometry3d transform = state.getGlobalLinkTransform(link);
  const Eigen::Quaterniond rotation(transform.linear());
  robot_controller::MotionPose pose;
  pose.x = transform.translation().x();
  pose.y = transform.translation().y();
  pose.z = transform.translation().z();
  pose.qx = rotation.x();
  pose.qy = rotation.y();
  pose.qz = rotation.z();
  pose.qw = rotation.w();
  return pose;
}

std::vector<double> positionsOf(const moveit::core::RobotState& state, const std::vector<std::string>& names)
{
  std::vector<double> values;
  values.reserve(names.size());
  for (const auto& name : names) {
    values.push_back(state.getVariablePosition(name));
  }
  return values;
}

moveit::core::RobotState makeState(const Scene& scene, const std::string& arm_state, const std::vector<double>* values)
{
  moveit::core::RobotState state(scene.model);
  state.setToDefaultValues();
  if (!state.setToDefaultValues(scene.grip, scene.grip_state)) {
    throw std::runtime_error("grip named state를 읽지 못했습니다.");
  }
  if (!arm_state.empty() && !state.setToDefaultValues(scene.arm, arm_state)) {
    throw std::runtime_error("arm named state를 읽지 못했습니다: " + arm_state);
  }
  if (values != nullptr) {
    state.setJointGroupPositions(scene.arm, *values);
  }
  state.update(true);
  return state;
}

void clampJoints(const Scene& scene, std::vector<double>& values)
{
  for (std::size_t index = 0; index < values.size(); ++index) {
    const auto& bounds = scene.model->getVariableBounds(scene.joint_names[index]);
    if (!bounds.position_bounded_) {
      continue;
    }
    values[index] = std::min(bounds.max_position_, std::max(bounds.min_position_, values[index]));
  }
}

bool withinLimits(const Scene& scene, const std::vector<double>& values)
{
  for (std::size_t index = 0; index < values.size(); ++index) {
    const auto& bounds = scene.model->getVariableBounds(scene.joint_names[index]);
    if (!bounds.position_bounded_) {
      continue;
    }
    if (values[index] < bounds.min_position_ - kPositionMargin || values[index] > bounds.max_position_ + kPositionMargin) {
      return false;
    }
  }
  return true;
}

struct CollisionSplit
{
  bool self_collision{false};
  bool full_collision{false};
};

bool stateColliding(const Scene& scene, const std::vector<double>& values)
{
  auto state = makeState(scene, "", &values);
  planning_scene_monitor::LockedPlanningSceneRO locked(scene.monitor);
  return locked->isStateColliding(state, "", false);
}

CollisionSplit collisionOf(const Scene& scene, const std::vector<double>& values)
{
  auto state = makeState(scene, "", &values);
  collision_detection::CollisionRequest request;
  collision_detection::CollisionResult self_result;
  collision_detection::CollisionResult full_result;
  planning_scene_monitor::LockedPlanningSceneRO locked(scene.monitor);
  locked->checkSelfCollision(request, self_result, state);
  locked->checkCollision(request, full_result, state);
  return {self_result.collision, full_result.collision};
}

bool projectPosition(const Scene& scene, std::vector<double>& values, const Eigen::Vector3d& goal, double& error)
{
  constexpr double kLambda = 1e-4;
  for (int iteration = 0; iteration < robot_controller::kPositionCorrectionIterations; ++iteration) {
    auto state = makeState(scene, "", &values);
    const Eigen::Vector3d current = state.getGlobalLinkTransform(scene.end_link).translation();
    const Eigen::Vector3d residual = goal - current;
    error = residual.norm();
    if (error <= robot_controller::kCapabilityPositionToleranceM) {
      return true;
    }
    Eigen::MatrixXd jacobian;
    if (!state.getJacobian(scene.arm, scene.end_link, Eigen::Vector3d::Zero(), jacobian)) {
      return false;
    }
    const Eigen::MatrixXd linear = jacobian.block(scene.linear_row, 0, 3, jacobian.cols());
    const Eigen::Matrix3d normal = (linear * linear.transpose()) + (kLambda * Eigen::Matrix3d::Identity());
    Eigen::VectorXd step = linear.transpose() * normal.ldlt().solve(residual);
    if (step.norm() > 0.25) {
      step *= 0.25 / step.norm();
    }
    for (std::size_t index = 0; index < values.size(); ++index) {
      values[index] += step(static_cast<Eigen::Index>(index));
    }
    clampJoints(scene, values);
    if (step.norm() < 1e-8) {
      break;
    }
  }
  auto state = makeState(scene, "", &values);
  error = (goal - state.getGlobalLinkTransform(scene.end_link).translation()).norm();
  return error <= robot_controller::kCapabilityPositionToleranceM;
}

robot_controller::CapabilityCandidate makeCandidate(
    const Scene& scene, const robot_controller::MotionPose& desired, const std::vector<double>& values,
    robot_controller::CandidateOrigin origin)
{
  robot_controller::CapabilityCandidate candidate;
  candidate.joints = values;
  candidate.origin = origin;
  auto state = makeState(scene, "", &values);
  candidate.executable = poseFromState(state, scene.end_link->getName());
  candidate.executable.source_index = desired.source_index;
  candidate.metrics = robot_controller::measureCapability(desired, candidate.executable);
  candidate.within_limits = withinLimits(scene, values);
  candidate.within_position_tolerance =
      candidate.metrics.position_error_m <= robot_controller::kCapabilityPositionToleranceM;
  candidate.colliding = candidate.within_limits && candidate.within_position_tolerance && stateColliding(scene, values);
  return candidate;
}

void addExactIk(
    const Scene& scene, const robot_controller::MotionPose& desired, const std::vector<double>& seed,
    std::vector<robot_controller::CapabilityCandidate>& candidates)
{
  ++g_projection_ik_calls;
  geometry_msgs::msg::Pose message;
  Eigen::Quaterniond rotation(desired.qw, desired.qx, desired.qy, desired.qz);
  rotation.normalize();
  message.position.x = desired.x;
  message.position.y = desired.y;
  message.position.z = desired.z;
  message.orientation.x = rotation.x();
  message.orientation.y = rotation.y();
  message.orientation.z = rotation.z();
  message.orientation.w = rotation.w();
  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes code;
  if (!scene.solver->searchPositionIK(message, seed, 0.0, solution, code) || solution.size() != seed.size()) {
    return;
  }
  candidates.push_back(makeCandidate(scene, desired, solution, robot_controller::CandidateOrigin::ExactIk));
}

std::array<std::array<double, 5>, 2> basisAt(const Scene& scene, const std::vector<double>& values)
{
  auto state = makeState(scene, "", &values);
  Eigen::MatrixXd jacobian;
  if (!state.getJacobian(scene.arm, scene.end_link, Eigen::Vector3d::Zero(), jacobian) || jacobian.cols() != 5) {
    throw std::runtime_error("position Jacobian을 읽지 못했습니다.");
  }
  double row_major[15];
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 5; ++column) {
      row_major[(row * 5) + column] = jacobian(scene.linear_row + row, column);
    }
  }
  return robot_controller::positionNullBasis(row_major);
}

std::vector<WaypointCapability> searchCase(
    const Scene& scene, const std::vector<robot_controller::MotionPose>& placed)
{
  const char* seed_names[] = {"init", "up", "down"};
  std::vector<WaypointCapability> results;
  for (const auto& desired : placed) {
    WaypointCapability item;
    item.source_index = desired.source_index;
    item.desired = desired;
    std::vector<robot_controller::CapabilityCandidate> candidates;
    const Eigen::Vector3d goal(desired.x, desired.y, desired.z);
    for (const char* seed_name : seed_names) {
      auto seed = positionsOf(makeState(scene, seed_name, nullptr), scene.joint_names);
      addExactIk(scene, desired, seed, candidates);
      double error = 0.0;
      auto positioned = seed;
      if (!projectPosition(scene, positioned, goal, error)) {
        continue;
      }
      candidates.push_back(makeCandidate(scene, desired, positioned, robot_controller::CandidateOrigin::PositionSeed));
      const auto basis = basisAt(scene, positioned);
      for (int ia = -robot_controller::kNullHalfCount; ia <= robot_controller::kNullHalfCount; ++ia) {
        for (int ib = -robot_controller::kNullHalfCount; ib <= robot_controller::kNullHalfCount; ++ib) {
          if (ia == 0 && ib == 0) {
            continue;
          }
          auto trial = positioned;
          for (std::size_t joint = 0; joint < trial.size(); ++joint) {
            trial[joint] += (static_cast<double>(ia) * robot_controller::kNullStepRad * basis[0][joint]) +
                            (static_cast<double>(ib) * robot_controller::kNullStepRad * basis[1][joint]);
          }
          clampJoints(scene, trial);
          if (!projectPosition(scene, trial, goal, error)) {
            continue;
          }
          candidates.push_back(makeCandidate(scene, desired, trial, robot_controller::CandidateOrigin::NullGrid));
        }
      }
    }
    item.selection = robot_controller::selectCapability(candidates);
    results.push_back(std::move(item));
  }
  std::vector<double> previous;
  for (std::size_t index = 0; index < results.size(); ++index) {
    if (index > 0) {
      results[index].desired_change =
          robot_controller::measureCapability(results[index - 1].desired, results[index].desired);
    }
    if (!results[index].selection.found) {
      continue;
    }
    if (!previous.empty()) {
      results[index].step = robot_controller::jointStep(previous, results[index].selection.selected.joints);
    }
    previous = results[index].selection.selected.joints;
  }
  return results;
}

int linearRow(const Scene& scene)
{
  auto state = makeState(scene, "init", nullptr);
  Eigen::MatrixXd jacobian;
  if (!state.getJacobian(scene.arm, scene.end_link, Eigen::Vector3d::Zero(), jacobian)) {
    return -1;
  }
  const auto seed = positionsOf(state, scene.joint_names);
  auto shifted = seed;
  shifted[1] += 1e-6;
  auto perturbed = makeState(scene, "", &shifted);
  const Eigen::Vector3d difference = perturbed.getGlobalLinkTransform(scene.end_link).translation() -
                                     state.getGlobalLinkTransform(scene.end_link).translation();
  const double scale = std::max(difference.norm(), 1e-12);
  const double top = (jacobian.block(0, 1, 3, 1) * 1e-6 - difference).norm() / scale;
  const double bottom = (jacobian.block(3, 1, 3, 1) * 1e-6 - difference).norm() / scale;
  if (top <= bottom && top < 0.05) {
    return 0;
  }
  if (bottom < 0.05) {
    return 3;
  }
  return -1;
}

std::vector<double> parseJoints(const std::string& text)
{
  std::vector<double> joints;
  std::stringstream stream(text);
  std::string field;
  while (std::getline(stream, field, ';')) {
    if (!field.empty()) {
      joints.push_back(std::stod(field));
    }
  }
  return joints;
}

std::vector<StoredProjection> loadReference(const std::string& path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Attempt #3 capability CSV를 열 수 없습니다: " + path);
  }
  std::string header;
  std::getline(input, header);
  const auto columns = splitCsv(header);
  auto indexOf = [&](const std::string& name) {
    const auto found = std::find(columns.begin(), columns.end(), name);
    if (found == columns.end()) {
      throw std::runtime_error("Attempt #3 CSV에 열이 없습니다: " + name);
    }
    return static_cast<std::size_t>(std::distance(columns.begin(), found));
  };
  const auto i_index = indexOf("source_index");
  const auto i_class = indexOf("capability_class");
  const auto i_joints = indexOf("joints");
  const auto i_pos = indexOf("position_error_m");
  const auto i_tool = indexOf("tool_axis_error_deg");
  const auto i_roll = indexOf("roll_error_deg");
  const auto i_full = indexOf("full_orientation_error_deg");
  std::vector<StoredProjection> rows;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }
    const auto fields = splitCsv(line);
    StoredProjection row;
    row.source_index = static_cast<std::size_t>(std::stoul(fields.at(i_index)));
    row.capability_class = fields.at(i_class);
    row.joints = parseJoints(fields.at(i_joints));
    row.position_error_m = std::stod(fields.at(i_pos));
    row.tool_axis_error_deg = std::stod(fields.at(i_tool));
    row.roll_error_deg = std::stod(fields.at(i_roll));
    row.full_orientation_error_deg = std::stod(fields.at(i_full));
    rows.push_back(std::move(row));
  }
  return rows;
}

Attempt4Row loadAttempt4(const std::string& path, const std::string& case_id)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Attempt #4 비교 CSV를 열 수 없습니다.");
  }
  std::string header;
  std::getline(input, header);
  const auto columns = splitCsv(header);
  auto indexOf = [&](const std::string& name) {
    const auto found = std::find(columns.begin(), columns.end(), name);
    if (found == columns.end()) {
      throw std::runtime_error("Attempt #4 CSV에 열이 없습니다: " + name);
    }
    return static_cast<std::size_t>(std::distance(columns.begin(), found));
  };
  const auto i_id = indexOf("case_id");
  const auto i_fraction = indexOf("attempt4_fraction");
  const auto i_planning = indexOf("attempt4_planning");
  std::string line;
  while (std::getline(input, line)) {
    const auto fields = splitCsv(line);
    if (fields.size() <= i_planning || fields[i_id] != case_id) {
      continue;
    }
    return {std::stod(fields[i_fraction]), fields[i_planning]};
  }
  throw std::runtime_error("Attempt #4 행을 찾지 못했습니다: " + case_id);
}

double measuredMaxVelocity(const std::vector<robot_controller::TimedJointPoint>& timed)
{
  double max_velocity = 0.0;
  for (std::size_t index = 1; index < timed.size(); ++index) {
    const double dt = timed[index].time_from_start_sec - timed[index - 1].time_from_start_sec;
    if (!(dt > 1e-4)) {
      return std::numeric_limits<double>::infinity();
    }
    double delta = 0.0;
    for (std::size_t joint = 0; joint < timed[index].joints.size(); ++joint) {
      delta = std::max(delta, std::abs(timed[index].joints[joint] - timed[index - 1].joints[joint]));
    }
    max_velocity = std::max(max_velocity, delta / dt);
  }
  return max_velocity;
}

JointCase buildJointCase(const Scene& scene, const std::vector<WaypointCapability>& rows)
{
  JointCase result;
  result.process_fidelity = robot_controller::kProcessFidelityNotYetAccepted;
  std::vector<std::vector<double>> waypoints;
  std::vector<std::size_t> sources;
  std::vector<double> position_errors;
  std::vector<double> tool_errors;
  std::vector<double> roll_errors;
  std::vector<double> full_errors;
  std::vector<double> step_values;
  for (const auto& row : rows) {
    if (!row.selection.found || !row.selection.selected.within_limits || row.selection.selected.colliding) {
      result.trajectory_class = robot_controller::kJointLimitFailure;
      return result;
    }
    const auto sealed = robot_controller::sealProjection(
        row.desired, row.selection.selected.executable, row.selection.selected.joints,
        robot_controller::capabilityClassName(row.selection.capability_class));
    position_errors.push_back(sealed.metrics.position_error_m);
    tool_errors.push_back(sealed.metrics.tool_axis_error_deg);
    roll_errors.push_back(std::abs(sealed.metrics.roll_error_deg));
    full_errors.push_back(sealed.metrics.full_orientation_error_deg);
    if (!waypoints.empty()) {
      const auto step = robot_controller::jointStep(waypoints.back(), sealed.joints);
      step_values.push_back(step.max_abs);
      if (step.branch_discontinuity) {
        ++result.branch_discontinuities;
      }
    }
    waypoints.push_back(sealed.joints);
    sources.push_back(sealed.source_index);
    result.sealed.push_back(sealed);
  }
  result.waypoint_count = static_cast<int>(waypoints.size());
  result.position = robot_controller::summarizeErrors(position_errors);
  result.tool = robot_controller::summarizeErrors(tool_errors);
  result.roll = robot_controller::summarizeErrors(roll_errors);
  result.full = robot_controller::summarizeErrors(full_errors);
  result.joint_step = robot_controller::summarizeErrors(step_values);
  const auto path = robot_controller::interpolateJointWaypoints(waypoints, robot_controller::kMaxJointInterpolationStepRad);
  const auto source_at = robot_controller::sourceIndexForSamples(sources, path.waypoint_sample_index, path.joints.size());
  if (path.joints.size() < 2 || source_at.size() != path.joints.size()) {
    result.trajectory_class = robot_controller::kTimeParameterizationFailure;
    return result;
  }
  std::vector<double> lower;
  std::vector<double> upper;
  for (const auto& name : scene.joint_names) {
    const auto& bounds = scene.model->getVariableBounds(name);
    lower.push_back(bounds.min_position_);
    upper.push_back(bounds.max_position_);
  }
  result.limit_violations = robot_controller::countOutsideBounds(path.joints, lower, upper, kPositionMargin);
  result.samples.reserve(path.joints.size());
  for (std::size_t index = 0; index < path.joints.size(); ++index) {
    const auto collision = collisionOf(scene, path.joints[index]);
    SampleRow sample;
    sample.sample_index = static_cast<int>(index);
    sample.source_index = source_at[index];
    sample.joints = path.joints[index];
    sample.within_limits = robot_controller::countOutsideBounds({path.joints[index]}, lower, upper, kPositionMargin) == 0;
    sample.self_collision = collision.self_collision;
    sample.full_collision = collision.full_collision;
    if (collision.self_collision) {
      ++result.self_collision;
    }
    if (collision.full_collision) {
      ++result.full_collision;
    }
    result.samples.push_back(std::move(sample));
  }
  result.interpolated_count = static_cast<int>(result.samples.size());
  const auto timed = robot_controller::retimeJointPath(
      path.joints, robot_controller::kJointVelocityCapRadS, robot_controller::kJointTimeStepFloorSec);
  result.timing_ok = timed.size() == path.joints.size();
  if (result.timing_ok) {
    moveit_msgs::msg::RobotTrajectory trajectory;
    trajectory.joint_trajectory.joint_names = scene.joint_names;
    for (std::size_t index = 0; index < timed.size(); ++index) {
      result.samples[index].time_from_start_sec = timed[index].time_from_start_sec;
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = timed[index].joints;
      const auto nanos = static_cast<int64_t>(std::llround(timed[index].time_from_start_sec * 1.0e9));
      point.time_from_start.sec = static_cast<int32_t>(nanos / 1000000000LL);
      point.time_from_start.nanosec = static_cast<uint32_t>(nanos % 1000000000LL);
      trajectory.joint_trajectory.points.push_back(std::move(point));
    }
    result.trajectory_built = trajectory.joint_trajectory.points.size() == timed.size() &&
                              robot_controller::armJointOrderMatches(trajectory.joint_trajectory.joint_names);
    result.duration_sec = timed.back().time_from_start_sec;
    result.max_velocity = measuredMaxVelocity(timed);
    result.max_acceleration = robot_controller::maxFiniteDifferenceAcceleration(timed);
    result.velocity_ok = robot_controller::jointVelocityWithin(timed, robot_controller::kJointVelocityCapRadS);
  }
  result.fk_waypoint_consistent = true;
  for (std::size_t index = 0; index < result.sealed.size(); ++index) {
    const int sample_index = path.waypoint_sample_index[index];
    const auto state = makeState(scene, "", &path.joints[static_cast<std::size_t>(sample_index)]);
    auto fk = poseFromState(state, scene.end_link->getName());
    fk.source_index = result.sealed[index].source_index;
    FkRow fk_row;
    fk_row.source_index = fk.source_index;
    fk_row.versus_projected = robot_controller::measureCapability(result.sealed[index].projected, fk);
    fk_row.versus_desired = robot_controller::measureCapability(result.sealed[index].desired, fk);
    result.max_fk_projected_position_m =
        std::max(result.max_fk_projected_position_m, fk_row.versus_projected.position_error_m);
    result.max_fk_projected_orientation_deg =
        std::max(result.max_fk_projected_orientation_deg, fk_row.versus_projected.full_orientation_error_deg);
    if (!robot_controller::fkMatchesProjectedPose(fk, result.sealed[index].projected, 1e-6, 1e-3)) {
      result.fk_waypoint_consistent = false;
    }
    result.fk_rows.push_back(fk_row);
  }
  for (std::size_t index = 0; index + 1 < path.waypoint_sample_index.size(); ++index) {
    const int begin = path.waypoint_sample_index[index];
    const int end = path.waypoint_sample_index[index + 1];
    std::vector<robot_controller::MotionPose> poses;
    for (int sample = begin; sample <= end; ++sample) {
      auto pose = poseFromState(makeState(scene, "", &path.joints[static_cast<std::size_t>(sample)]), scene.end_link->getName());
      poses.push_back(pose);
    }
    SegmentRow segment;
    segment.from_source = sources[index];
    segment.to_source = sources[index + 1];
    segment.step = robot_controller::jointStep(waypoints[index], waypoints[index + 1]);
    segment.intervals = end - begin;
    segment.cartesian = robot_controller::measureCartesianSegment(poses);
    result.max_chord_deviation_m = std::max(result.max_chord_deviation_m, segment.cartesian.max_chord_deviation_m);
    result.max_tool_axis_excursion_deg =
        std::max(result.max_tool_axis_excursion_deg, segment.cartesian.tool_axis_excursion_deg);
    result.max_cartesian_path_length_m =
        std::max(result.max_cartesian_path_length_m, segment.cartesian.path_length_m);
    result.segments.push_back(segment);
  }
  const auto separated = robot_controller::separateJointFeasibility(
      result.sealed.empty() ? robot_controller::OrientationMetrics{} : result.sealed.front().metrics,
      robot_controller::classifyJointTrajectory(
          result.joint_step.max, result.limit_violations, result.full_collision, result.timing_ok && result.trajectory_built,
          result.velocity_ok));
  result.trajectory_class = separated.trajectory_class;
  result.process_fidelity = separated.process_fidelity;
  return result;
}

std::string jointsText(const std::vector<double>& joints)
{
  std::ostringstream text;
  text << std::setprecision(17);
  for (std::size_t index = 0; index < joints.size(); ++index) {
    if (index > 0) {
      text << ';';
    }
    text << joints[index];
  }
  return text.str();
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("joint_trajectory_dry", options);
  if (node->get_parameter("execute_trajectory").as_bool()) {
    RCLCPP_ERROR(node->get_logger(), "execute_trajectory는 false여야 합니다.");
    rclcpp::shutdown();
    return 2;
  }
  const auto output_dir = node->get_parameter("output_dir").as_string();
  int exit_code = 1;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread spinner;
  bool spinning = false;
  auto stop_spin = [&]() {
    if (spinning) {
      executor.cancel();
      spinner.join();
      spinning = false;
    }
  };
  try {
    if (std::abs(node->get_parameter("eef_step").as_double() - kEefStep) > 1e-12 ||
        std::abs(node->get_parameter("jump_threshold").as_double() - kJumpThreshold) > 1e-12 ||
        std::abs(node->get_parameter("min_fraction").as_double() - robot_controller::kMinCartesianFraction) > 1e-12 ||
        std::abs(node->get_parameter("max_joint_velocity").as_double() - robot_controller::kJointVelocityCapRadS) > 1e-12 ||
        std::abs(node->get_parameter("joint_time_step_floor_sec").as_double() - robot_controller::kJointTimeStepFloorSec) >
            1e-12 ||
        std::abs(
            node->get_parameter("max_joint_interpolation_step_rad").as_double() -
            robot_controller::kMaxJointInterpolationStepRad) > 1e-12 ||
        node->get_parameter("runs").as_int() != 3 ||
        std::abs(node->get_parameter("position_tolerance_m").as_double() - robot_controller::kCapabilityPositionToleranceM) >
            1e-12 ||
        node->get_parameter("null_half_count").as_int() != robot_controller::kNullHalfCount ||
        std::abs(node->get_parameter("null_step_rad").as_double() - robot_controller::kNullStepRad) > 1e-12 ||
        node->get_parameter("position_correction_iterations").as_int() != robot_controller::kPositionCorrectionIterations) {
      throw std::runtime_error("보간, 속도 상한, 또는 projection 파라미터가 고정값과 다릅니다.");
    }
    if (sha256File(node->get_parameter("snapshot_pcd").as_string()) != kSnapshotSha ||
        sha256File(node->get_parameter("waypoints_csv").as_string()) != kWaypointSha) {
      throw std::runtime_error("snapshot 또는 waypoint SHA256이 P-004와 다릅니다.");
    }
    std::vector<std::vector<StoredProjection>> reference;
    std::vector<Attempt4Row> attempt4;
    for (const auto& frozen : kFrozenCases) {
      reference.push_back(loadReference(
          node->get_parameter("attempt03_dir").as_string() + "/" +
          (std::string("case-0") + std::string(frozen.id).back() + "-capability.csv")));
      attempt4.push_back(loadAttempt4(node->get_parameter("attempt04_compare").as_string(), frozen.id));
    }
    auto camera_poses = loadWaypoints(node->get_parameter("waypoints_csv").as_string());
    const auto camera_copy = camera_poses;
    const auto pairs = assessPairs(camera_poses, node->get_parameter("max_segment_gap").as_double());
    const auto validation = robot_controller::validateMotionInput(
        camera_poses, pairs, node->get_parameter("max_orientation_step_deg").as_double());
    if (!validation.error.empty()) {
      throw std::runtime_error(validation.error);
    }
    std::string window_error;
    const auto windows = robot_controller::selectPlanningWindows(
        camera_poses, validation, static_cast<std::size_t>(node->get_parameter("window_count").as_int()),
        node->get_parameter("max_orientation_step_deg").as_double(), window_error);
    if (!window_error.empty() || windows.size() != 3) {
      throw std::runtime_error(window_error.empty() ? "창 개수가 다릅니다." : window_error);
    }
    for (std::size_t index = 0; index < 3; ++index) {
      if (windows[index].start_index != kFrozenCases[index].start || windows[index].end_index != kFrozenCases[index].end) {
        throw std::runtime_error("계획 창이 attempt 1과 다릅니다.");
      }
    }
    const auto urdf_xml = readText(node->get_parameter("urdf_path").as_string());
    const auto srdf_xml = readText(node->get_parameter("srdf_path").as_string());
    node->declare_parameter("robot_description", urdf_xml);
    node->declare_parameter("robot_description_semantic", srdf_xml);
    urdf::Model urdf_model;
    if (!urdf_model.initString(urdf_xml)) {
      throw std::runtime_error("URDF를 읽지 못했습니다.");
    }
    std::vector<std::string> movable;
    for (const auto& item : urdf_model.joints_) {
      if (item.second->type == urdf::Joint::FIXED || item.second->type == urdf::Joint::UNKNOWN) {
        continue;
      }
      if (item.first.find("Arm6") != std::string::npos) {
        throw std::runtime_error("Arm6 조인트가 모델에 있습니다.");
      }
      movable.push_back(item.first);
    }
    std::sort(movable.begin(), movable.end());
    JointStatePump pump(node, movable);
    executor.add_node(node);
    spinner = std::thread([&executor]() { executor.spin(); });
    spinning = true;
    pump.publish();
    auto moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(node);
    Scene scene;
    scene.model = moveit_cpp->getRobotModel();
    scene.arm = scene.model->getJointModelGroup("arm_group");
    scene.grip = scene.model->getJointModelGroup("grip_group");
    scene.end_link = scene.model->getLinkModel("Arm5_Link");
    scene.grip_state = "open";
    scene.monitor = moveit_cpp->getPlanningSceneMonitor();
    if (scene.arm == nullptr || scene.grip == nullptr || scene.end_link == nullptr || !scene.arm->getSolverInstance()) {
      throw std::runtime_error("arm_group 또는 solver가 없습니다.");
    }
    scene.solver = scene.arm->getSolverInstance().get();
    scene.joint_names = scene.solver->getJointNames();
    if (!robot_controller::armJointOrderMatches(scene.joint_names) ||
        scene.arm->getVariableNames() != scene.joint_names || scene.arm->getActiveJointModels().size() != 5) {
      throw std::runtime_error("runtime joint ordering이 Arm1–Arm5가 아닙니다.");
    }
    scene.linear_row = linearRow(scene);
    if (scene.linear_row < 0) {
      throw std::runtime_error("position Jacobian 행을 확인하지 못했습니다.");
    }
    const auto anchor = poseFromState(makeState(scene, "init", nullptr), "Arm5_Link");
    std::vector<std::vector<robot_controller::MotionPose>> placed_cases;
    for (const auto& frozen : kFrozenCases) {
      const auto computed = robot_controller::rigidPlacement(camera_poses[frozen.start], anchor);
      double worst = 0.0;
      for (int axis = 0; axis < 3; ++axis) {
        worst = std::max(worst, std::abs(computed.translation[axis] - frozen.translation[axis]));
      }
      for (int cell = 0; cell < 9; ++cell) {
        worst = std::max(worst, std::abs(computed.rotation[cell] - frozen.rotation[cell]));
      }
      if (worst >= 1e-9 || std::abs(computed.scale - 1.0) > 1e-12) {
        throw std::runtime_error("synthetic placement가 attempt 1과 다릅니다.");
      }
      robot_controller::RigidPlacement used = computed;
      for (int axis = 0; axis < 3; ++axis) {
        used.translation[axis] = frozen.translation[axis];
      }
      for (int cell = 0; cell < 9; ++cell) {
        used.rotation[cell] = frozen.rotation[cell];
      }
      std::vector<robot_controller::MotionPose> placed;
      for (std::size_t source = frozen.start; source <= frozen.end; ++source) {
        placed.push_back(robot_controller::applyRigidPlacement(used, camera_poses[source]));
      }
      if (placed.size() != 11) {
        throw std::runtime_error("waypoint를 삭제하면 안 됩니다.");
      }
      placed_cases.push_back(std::move(placed));
    }
    std::vector<std::vector<std::vector<WaypointCapability>>> projections;
    double joint_diff = 0.0;
    double metric_diff = 0.0;
    bool projection_match = true;
    for (int run = 0; run < 3; ++run) {
      std::vector<std::vector<WaypointCapability>> cases;
      for (std::size_t case_index = 0; case_index < placed_cases.size(); ++case_index) {
        auto rows = searchCase(scene, placed_cases[case_index]);
        if (rows.size() != reference[case_index].size()) {
          projection_match = false;
        }
        for (std::size_t index = 0; index < rows.size() && index < reference[case_index].size(); ++index) {
          const auto& stored = reference[case_index][index];
          const auto& row = rows[index];
          if (!row.selection.found ||
              robot_controller::capabilityClassName(row.selection.capability_class) != stored.capability_class ||
              row.selection.selected.joints.size() != stored.joints.size()) {
            projection_match = false;
            continue;
          }
          for (std::size_t joint = 0; joint < stored.joints.size(); ++joint) {
            joint_diff = std::max(joint_diff, std::abs(row.selection.selected.joints[joint] - stored.joints[joint]));
          }
          metric_diff = std::max(
              metric_diff, std::abs(row.selection.selected.metrics.position_error_m - stored.position_error_m));
          metric_diff = std::max(
              metric_diff, std::abs(row.selection.selected.metrics.tool_axis_error_deg - stored.tool_axis_error_deg));
          metric_diff = std::max(
              metric_diff, std::abs(row.selection.selected.metrics.roll_error_deg - stored.roll_error_deg));
          metric_diff = std::max(
              metric_diff,
              std::abs(row.selection.selected.metrics.full_orientation_error_deg - stored.full_orientation_error_deg));
        }
        cases.push_back(std::move(rows));
      }
      projections.push_back(std::move(cases));
    }
    if (joint_diff > 1e-8 || metric_diff > 1e-6) {
      projection_match = false;
    }
    {
      std::ofstream reproduction(output_dir + "/projection-reproduction.txt");
      reproduction << std::setprecision(17);
      reproduction << "position_tolerance_m " << robot_controller::kCapabilityPositionToleranceM << "\n";
      reproduction << "null_half_count " << robot_controller::kNullHalfCount << "\n";
      reproduction << "null_step_rad " << robot_controller::kNullStepRad << "\n";
      reproduction << "position_correction_iterations " << robot_controller::kPositionCorrectionIterations << "\n";
      reproduction << "seeds init up down\n";
      reproduction << "attempt3_match " << (projection_match ? "yes" : "no") << "\n";
      reproduction << "max_joint_diff_rad " << joint_diff << "\n";
      reproduction << "max_metric_diff " << metric_diff << "\n";
      reproduction << "joint_tolerance_rad 1e-8\nmetric_tolerance 1e-6\n";
      reproduction << "desired_changed no\n";
      reproduction << "pose_to_ik_roundtrip_after_projection 0\n";
    }
    if (!projection_match) {
      stop_spin();
      std::ofstream json(output_dir + "/joint-trajectory-results.json");
      json << "{\n  \"status\": \"FAIL\",\n  \"reason\": \"attempt 3 projection mismatch\",\n"
           << "  \"process_fidelity\": \"" << robot_controller::kProcessFidelityNotYetAccepted << "\"\n}\n";
      std::cout << "P-005 ATTEMPT #5 FAIL projection mismatch\n";
      rclcpp::shutdown();
      return 1;
    }
    const int projection_ik_calls = g_projection_ik_calls;
    std::vector<std::vector<JointCase>> runs;
    for (const auto& cases : projections) {
      std::vector<JointCase> built;
      for (const auto& rows : cases) {
        built.push_back(buildJointCase(scene, rows));
        RCLCPP_INFO(
            node->get_logger(), "%s states %d collisions %d %s", built.back().trajectory_class.c_str(),
            built.back().interpolated_count, built.back().full_collision, built.back().process_fidelity.c_str());
      }
      runs.push_back(std::move(built));
    }
    if (g_projection_ik_calls != projection_ik_calls || kTrajectoryIkCalls != 0) {
      throw std::runtime_error("trajectory 구성 중에 IK를 다시 호출했습니다.");
    }
    bool same = runs.size() == 3;
    for (std::size_t run = 1; same && run < runs.size(); ++run) {
      for (std::size_t index = 0; index < 3; ++index) {
        const auto& first = runs[0][index];
        const auto& other = runs[run][index];
        if (first.trajectory_class != other.trajectory_class || first.interpolated_count != other.interpolated_count ||
            first.limit_violations != other.limit_violations || first.full_collision != other.full_collision ||
            first.self_collision != other.self_collision ||
            std::abs(first.duration_sec - other.duration_sec) > 1e-9 ||
            std::abs(first.max_velocity - other.max_velocity) > 1e-9) {
          same = false;
        }
        if (first.sealed.size() != other.sealed.size()) {
          same = false;
          continue;
        }
        for (std::size_t waypoint = 0; waypoint < first.sealed.size(); ++waypoint) {
          for (std::size_t joint = 0; joint < first.sealed[waypoint].joints.size(); ++joint) {
            if (std::abs(first.sealed[waypoint].joints[joint] - other.sealed[waypoint].joints[joint]) > 1e-12) {
              same = false;
            }
          }
        }
      }
    }
    bool feasible = same;
    for (const auto& item : runs[0]) {
      if (item.trajectory_class != robot_controller::kJointTrajectoryFeasible ||
          item.process_fidelity != robot_controller::kProcessFidelityNotYetAccepted || !item.fk_waypoint_consistent) {
        feasible = false;
      }
    }
    const std::string status = feasible ? "PASS_CANDIDATE" : "FAIL";
    {
      std::ofstream config(output_dir + "/joint-trajectory-config.yaml");
      config << std::setprecision(17);
      config << "max_joint_interpolation_step_rad: " << robot_controller::kMaxJointInterpolationStepRad << "\n";
      config << "interpolation_rationale: \"0.05 rad is one tenth of the 0.5 rad branch threshold and splits the Attempt 3 max step of 0.4 rad into 8 intervals. Locked before this run.\"\n";
      config << "max_joint_velocity: " << robot_controller::kJointVelocityCapRadS << "\n";
      config << "joint_time_step_floor_sec: " << robot_controller::kJointTimeStepFloorSec << "\n";
      config << "time_parameterization: motion_planner_node_retime\n";
      config << "hardware_acceleration_acceptance: false\n";
      config << "vendor_acceleration_limits: disabled\n";
      config << "branch_discontinuity_rad: " << robot_controller::kBranchDiscontinuityRad << "\n";
      config << "execute_trajectory: false\n";
      std::ofstream waypoints_csv(output_dir + "/projected-joint-waypoints.csv");
      waypoints_csv << std::setprecision(17);
      waypoints_csv << "case_id,source_index,capability_class,Arm1_Joint,Arm2_Joint,Arm3_Joint,Arm4_Joint,Arm5_Joint,"
                       "position_error_m,tool_axis_error_deg,roll_error_deg,full_orientation_error_deg\n";
      std::ofstream deltas(output_dir + "/joint-deltas.csv");
      deltas << std::setprecision(17);
      deltas << "case_id,from_source,to_source,delta_arm1,delta_arm2,delta_arm3,delta_arm4,delta_arm5,max_abs,"
                "branch_discontinuity\n";
      std::ofstream interpolation(output_dir + "/joint-interpolation.csv");
      interpolation << std::setprecision(17);
      interpolation << "case_id,from_source,to_source,intervals,max_joint_step_rad\n";
      std::ofstream fk_csv(output_dir + "/fk-validation.csv");
      fk_csv << std::setprecision(17);
      fk_csv << "case_id,source_index,position_error_vs_projected_m,full_orientation_error_vs_projected_deg,"
                "position_error_vs_desired_m,tool_axis_error_vs_desired_deg,roll_error_vs_desired_deg,"
                "full_orientation_error_vs_desired_deg\n";
      std::ofstream cartesian(output_dir + "/cartesian-deviation.csv");
      cartesian << std::setprecision(17);
      cartesian << "case_id,from_source,to_source,cartesian_path_length_m,chord_length_m,max_chord_deviation_m,"
                   "tool_axis_excursion_deg\n";
      std::ofstream compare(output_dir + "/attempt-04-vs-attempt-05.csv");
      compare << std::setprecision(17);
      compare << "case_id,attempt4_fraction,attempt4_planning,attempt5_trajectory,attempt5_fidelity,"
                 "interpolated_states,max_joint_step_rad,collision_invalid,joint_limit_violations\n";
      for (std::size_t case_index = 0; case_index < 3; ++case_index) {
        const auto& item = runs[0][case_index];
        std::ofstream trajectory_csv(output_dir + "/case-0" + std::to_string(case_index + 1) + "-joint-trajectory.csv");
        trajectory_csv << std::setprecision(17);
        trajectory_csv << "sample_index,time_from_start_sec,source_index,Arm1_Joint,Arm2_Joint,Arm3_Joint,Arm4_Joint,"
                          "Arm5_Joint,within_limits,self_collision,planning_scene_collision\n";
        for (const auto& sealed : item.sealed) {
          waypoints_csv << kFrozenCases[case_index].id << ',' << sealed.source_index << ',' << sealed.capability_class;
          for (double joint : sealed.joints) {
            waypoints_csv << ',' << joint;
          }
          waypoints_csv << ',' << sealed.metrics.position_error_m << ',' << sealed.metrics.tool_axis_error_deg << ','
                        << sealed.metrics.roll_error_deg << ',' << sealed.metrics.full_orientation_error_deg << '\n';
        }
        for (const auto& segment : item.segments) {
          deltas << kFrozenCases[case_index].id << ',' << segment.from_source << ',' << segment.to_source;
          for (double delta : segment.step.delta) {
            deltas << ',' << delta;
          }
          deltas << ',' << segment.step.max_abs << ',' << (segment.step.branch_discontinuity ? "yes" : "no") << '\n';
          interpolation << kFrozenCases[case_index].id << ',' << segment.from_source << ',' << segment.to_source << ','
                        << segment.intervals << ',' << segment.step.max_abs << '\n';
          cartesian << kFrozenCases[case_index].id << ',' << segment.from_source << ',' << segment.to_source << ','
                    << segment.cartesian.path_length_m << ',' << segment.cartesian.chord_length_m << ','
                    << segment.cartesian.max_chord_deviation_m << ',' << segment.cartesian.tool_axis_excursion_deg
                    << '\n';
        }
        for (const auto& sample : item.samples) {
          trajectory_csv << sample.sample_index << ',' << sample.time_from_start_sec << ',';
          if (sample.source_index == std::numeric_limits<std::size_t>::max()) {
            trajectory_csv << "";
          } else {
            trajectory_csv << sample.source_index;
          }
          for (double joint : sample.joints) {
            trajectory_csv << ',' << joint;
          }
          trajectory_csv << ',' << (sample.within_limits ? "yes" : "no") << ','
                         << (sample.self_collision ? "yes" : "no") << ','
                         << (sample.full_collision ? "yes" : "no") << '\n';
        }
        for (const auto& fk_row : item.fk_rows) {
          fk_csv << kFrozenCases[case_index].id << ',' << fk_row.source_index << ','
                 << fk_row.versus_projected.position_error_m << ','
                 << fk_row.versus_projected.full_orientation_error_deg << ','
                 << fk_row.versus_desired.position_error_m << ',' << fk_row.versus_desired.tool_axis_error_deg << ','
                 << fk_row.versus_desired.roll_error_deg << ',' << fk_row.versus_desired.full_orientation_error_deg
                 << '\n';
        }
        compare << kFrozenCases[case_index].id << ',' << attempt4[case_index].fraction << ','
                << attempt4[case_index].planning << ',' << item.trajectory_class << ',' << item.process_fidelity << ','
                << item.interpolated_count << ',' << item.joint_step.max << ',' << item.full_collision << ','
                << item.limit_violations << '\n';
      }
      std::ofstream collision(output_dir + "/collision-validation.txt");
      collision << "self_collision_check checkSelfCollision\n";
      collision << "planning_scene_collision_check checkCollision\n";
      collision << "interpolated_states_checked yes\n";
      for (std::size_t run = 0; run < runs.size(); ++run) {
        for (std::size_t index = 0; index < 3; ++index) {
          collision << "run " << (run + 1) << ' ' << kFrozenCases[index].id << " self "
                    << runs[run][index].self_collision << " planning_scene " << runs[run][index].full_collision
                    << "\n";
        }
      }
      std::ofstream limits(output_dir + "/joint-limit-validation.txt");
      limits << std::setprecision(17);
      limits << "position_margin_rad " << kPositionMargin << "\n";
      limits << "intermediate_states_checked yes\n";
      for (std::size_t run = 0; run < runs.size(); ++run) {
        for (std::size_t index = 0; index < 3; ++index) {
          limits << "run " << (run + 1) << ' ' << kFrozenCases[index].id << " violations "
                 << runs[run][index].limit_violations << "\n";
        }
      }
      std::ofstream velocity(output_dir + "/velocity-validation.txt");
      velocity << std::setprecision(17);
      velocity << "time_parameterization motion_planner_node_retime\n";
      velocity << "max_joint_velocity_rad_s " << robot_controller::kJointVelocityCapRadS << "\n";
      velocity << "min_dt_sec " << robot_controller::kJointTimeStepFloorSec << "\n";
      velocity << "hardware_acceleration_acceptance false\n";
      velocity << "vendor_acceleration_limits disabled\n";
      for (std::size_t run = 0; run < runs.size(); ++run) {
        for (std::size_t index = 0; index < 3; ++index) {
          const auto& item = runs[run][index];
          velocity << "run " << (run + 1) << ' ' << kFrozenCases[index].id << " max_velocity " << item.max_velocity
                   << " finite_difference_acceleration " << item.max_acceleration << " duration_sec "
                   << item.duration_sec << " velocity_ok " << (item.velocity_ok ? "yes" : "no") << "\n";
        }
      }
      std::ofstream reproducibility(output_dir + "/reproducibility.txt");
      reproducibility << std::setprecision(17);
      reproducibility << "runs " << runs.size() << "\n";
      reproducibility << "equal " << (same ? "yes" : "no") << "\n";
      reproducibility << "time_tolerance_sec 1e-9\n";
      for (std::size_t run = 0; run < runs.size(); ++run) {
        reproducibility << "run " << (run + 1);
        for (std::size_t index = 0; index < 3; ++index) {
          const auto& item = runs[run][index];
          reproducibility << ' ' << kFrozenCases[index].id << " states " << item.interpolated_count << " limits "
                         << item.limit_violations << " collisions " << item.full_collision << ' '
                         << item.trajectory_class << " duration " << item.duration_sec;
        }
        reproducibility << "\n";
      }
      std::ofstream svg(output_dir + "/desired-vs-joint-trajectory.svg");
      svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"980\" height=\"360\" viewBox=\"0 0 980 360\">\n";
      svg << "<rect width=\"980\" height=\"360\" fill=\"#f7f4ee\"/>\n";
      svg << "<text x=\"24\" y=\"32\" font-family=\"sans-serif\" font-size=\"18\" fill=\"#1c1917\">joint trajectory chord deviation and tool-axis excursion</text>\n";
      const char* colors[] = {"#b45309", "#1d4ed8", "#44403c"};
      for (std::size_t case_index = 0; case_index < 3; ++case_index) {
        std::ostringstream chord_points;
        std::ostringstream tool_points;
        const auto& item = runs[0][case_index];
        for (std::size_t index = 0; index < item.segments.size(); ++index) {
          const double x = 40.0 + (static_cast<double>(case_index) * 320.0) + (static_cast<double>(index) * 28.0);
          const double chord_y = 300.0 - std::min(item.segments[index].cartesian.max_chord_deviation_m * 4000.0, 240.0);
          const double tool_y = 300.0 - std::min(item.segments[index].cartesian.tool_axis_excursion_deg, 240.0);
          chord_points << x << ',' << chord_y << ' ';
          tool_points << x << ',' << tool_y << ' ';
        }
        svg << "<polyline fill=\"none\" stroke=\"" << colors[case_index] << "\" stroke-width=\"2\" points=\""
            << chord_points.str() << "\"/>\n";
        svg << "<polyline fill=\"none\" stroke=\"" << colors[case_index]
            << "\" stroke-width=\"1\" stroke-dasharray=\"4 3\" points=\"" << tool_points.str() << "\"/>\n";
        svg << "<text x=\"" << (40 + case_index * 320) << "\" y=\"340\" font-family=\"sans-serif\" font-size=\"13\" fill=\""
            << colors[case_index] << "\">" << kFrozenCases[case_index].id << "</text>\n";
      }
      svg << "</svg>\n";
      std::ofstream json(output_dir + "/joint-trajectory-results.json");
      json << std::setprecision(17);
      json << "{\n  \"status\": \"" << status << "\",\n";
      json << "  \"source\": {\"snapshot_sha256\": \"" << kSnapshotSha << "\", \"waypoints_sha256\": \"" << kWaypointSha
           << "\", \"cases\": [\"61-71\", \"312-322\", \"572-582\"], \"desired_immutable\": true},\n";
      json << "  \"projection\": {\"match\": " << (projection_match ? "true" : "false") << ", \"max_joint_diff_rad\": "
           << joint_diff << ", \"max_metric_diff\": " << metric_diff
           << ", \"position_tolerance_m\": " << robot_controller::kCapabilityPositionToleranceM
           << ", \"null_half_count\": " << robot_controller::kNullHalfCount
           << ", \"null_step_rad\": " << robot_controller::kNullStepRad << "},\n";
      json << "  \"joint_waypoints\": {\"count\": 33, \"ordering\": [\"Arm1_Joint\", \"Arm2_Joint\", \"Arm3_Joint\", "
              "\"Arm4_Joint\", \"Arm5_Joint\"], \"pose_to_ik_roundtrip\": 0},\n";
      json << "  \"interpolation\": {\"max_joint_interpolation_step_rad\": "
           << robot_controller::kMaxJointInterpolationStepRad << ", \"locked_before_run\": true},\n";
      json << "  \"collision\": {\"self_check\": \"checkSelfCollision\", \"scene_check\": \"checkCollision\"},\n";
      json << "  \"joint_limits\": {\"margin_rad\": " << kPositionMargin << "},\n";
      json << "  \"time_parameterization\": {\"method\": \"motion_planner_node_retime\", \"floor_sec\": "
           << robot_controller::kJointTimeStepFloorSec << "},\n";
      json << "  \"velocity\": {\"cap_rad_s\": " << robot_controller::kJointVelocityCapRadS
           << ", \"hardware_acceleration_acceptance\": false},\n";
      json << "  \"process_fidelity\": \"" << robot_controller::kProcessFidelityNotYetAccepted << "\",\n";
      json << "  \"safety\": {\"execute_calls\": " << kExecuteCalls << ", \"controller_sends\": " << kControllerSends
           << ", \"trajectory_ik_calls\": " << kTrajectoryIkCalls << ", \"projection_ik_calls\": " << projection_ik_calls
           << ", \"camera_robot_calibration\": 0, \"fake_tf\": 0, \"actual_robot_commands\": 0},\n";
      json << "  \"cases\": [\n";
      for (std::size_t case_index = 0; case_index < 3; ++case_index) {
        const auto& item = runs[0][case_index];
        json << "    {\"case_id\": \"" << kFrozenCases[case_index].id << "\",\n";
        json << "     \"joint_waypoints\": " << item.waypoint_count << ", \"interpolated_states\": "
             << item.interpolated_count << ",\n";
        json << "     \"joint_step\": {\"mean\": " << item.joint_step.mean << ", \"p95\": " << item.joint_step.p95
             << ", \"max\": " << item.joint_step.max << ", \"branch_discontinuities\": " << item.branch_discontinuities
             << "},\n";
        json << "     \"collision\": {\"self\": " << item.self_collision << ", \"planning_scene\": " << item.full_collision
             << "},\n";
        json << "     \"joint_limits\": {\"violations\": " << item.limit_violations << "},\n";
        json << "     \"time_parameterization\": {\"ok\": " << (item.timing_ok ? "true" : "false")
             << ", \"duration_sec\": " << item.duration_sec << "},\n";
        json << "     \"velocity\": {\"max_rad_s\": " << item.max_velocity
             << ", \"finite_difference_acceleration_rad_s2\": " << item.max_acceleration
             << ", \"within_cap\": " << (item.velocity_ok ? "true" : "false") << "},\n";
        json << "     \"fk_validation\": {\"max_position_error_vs_projected_m\": " << item.max_fk_projected_position_m
             << ", \"max_orientation_error_vs_projected_deg\": " << item.max_fk_projected_orientation_deg
             << ", \"consistent\": " << (item.fk_waypoint_consistent ? "true" : "false") << "},\n";
        json << "     \"cartesian_deviation\": {\"max_chord_m\": " << item.max_chord_deviation_m
             << ", \"max_tool_axis_excursion_deg\": " << item.max_tool_axis_excursion_deg
             << ", \"max_segment_path_length_m\": " << item.max_cartesian_path_length_m << "},\n";
        json << "     \"projection_fidelity\": {\"position_mean\": " << item.position.mean << ", \"position_p95\": "
             << item.position.p95 << ", \"position_max\": " << item.position.max << ", \"tool_mean\": " << item.tool.mean
             << ", \"tool_p95\": " << item.tool.p95 << ", \"tool_max\": " << item.tool.max << ", \"roll_abs_mean\": "
             << item.roll.mean << ", \"roll_abs_p95\": " << item.roll.p95 << ", \"roll_abs_max\": " << item.roll.max
             << ", \"full_mean\": " << item.full.mean << ", \"full_p95\": " << item.full.p95 << ", \"full_max\": "
             << item.full.max << "},\n";
        json << "     \"attempt4\": {\"fraction\": " << attempt4[case_index].fraction << ", \"planning\": \""
             << attempt4[case_index].planning << "\"},\n";
        json << "     \"trajectory_class\": \"" << item.trajectory_class << "\",\n";
        json << "     \"process_fidelity\": \"" << item.process_fidelity << "\"}"
             << (case_index + 1 == 3 ? "\n" : ",\n");
      }
      json << "  ]\n}\n";
    }
    stop_spin();
    for (std::size_t index = 0; index < camera_poses.size(); ++index) {
      if (camera_poses[index].x != camera_copy[index].x || camera_poses[index].qw != camera_copy[index].qw ||
          camera_poses[index].nx != camera_copy[index].nx) {
        throw std::runtime_error("desired geometry가 바뀌었습니다.");
      }
    }
    std::cout << "P-005 ATTEMPT #5 " << status << " fidelity " << robot_controller::kProcessFidelityNotYetAccepted
              << "\n";
    exit_code = 0;
  } catch (const std::exception& error) {
    stop_spin();
    std::ofstream json(output_dir + "/joint-trajectory-results.json");
    json << "{\n  \"status\": \"FAIL\",\n  \"error\": \"" << error.what() << "\"\n}\n";
    std::cout << "P-005 ATTEMPT #5 FAIL\n" << error.what() << "\n";
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
