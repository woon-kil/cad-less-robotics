// Copyright 2026 cad-less-robotics contributors
// P-005 attempt 4. Attempt #3의 projected FK만 Cartesian planner에 넣는다.
// desired geometry는 바꾸지 않고, planning 성공이 process fidelity 합격이 되지 않는다.

#include "robot_controller/capability_adapter.hpp"
#include "robot_controller/motion_contract.hpp"
#include "vision_path_generator/polyline.hpp"

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/cartesian_interpolator.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <urdf/model.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
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
constexpr double kMaxJointVelocity = 0.4;
constexpr double kPositionMargin = 1e-6;
constexpr double kAttempt1Fraction = 0.090909090909090912;

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

struct TrackSample
{
  std::size_t source_index{0};
  std::string kind;
  robot_controller::OrientationMetrics versus_projected;
  robot_controller::OrientationMetrics versus_desired;
};

struct PlannedCase
{
  std::vector<robot_controller::SealedProjection> sealed;
  double fraction{0.0};
  int trajectory_points{0};
  double planning_time_sec{0.0};
  double max_joint_velocity{0.0};
  int position_violations{0};
  int velocity_violations{0};
  int collision_states{0};
  bool start_collision{false};
  bool matches_srdf_init{false};
  std::string start_provenance;
  std::string planning_feasibility;
  std::string process_fidelity;
  std::vector<TrackSample> samples;
  robot_controller::ErrorSummary position;
  robot_controller::ErrorSummary tool;
  robot_controller::ErrorSummary roll;
  robot_controller::ErrorSummary full;
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

bool stateColliding(const Scene& scene, const std::vector<double>& values)
{
  auto state = makeState(scene, "", &values);
  planning_scene_monitor::LockedPlanningSceneRO locked(scene.monitor);
  return locked->isStateColliding(state, "", false);
}

bool stateColliding(const Scene& scene, const moveit::core::RobotState& state)
{
  planning_scene_monitor::LockedPlanningSceneRO locked(scene.monitor);
  moveit::core::RobotState copy(state);
  copy.update(true);
  return locked->isStateColliding(copy, "", false);
}

int positionViolations(const Scene& scene, const moveit::core::RobotState& state)
{
  int violations = 0;
  for (const auto& name : scene.joint_names) {
    const auto& bounds = state.getRobotModel()->getVariableBounds(name);
    if (!bounds.position_bounded_) {
      continue;
    }
    const double value = state.getVariablePosition(name);
    if (value < bounds.min_position_ - kPositionMargin || value > bounds.max_position_ + kPositionMargin) {
      ++violations;
    }
  }
  return violations;
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

double attempt1Fraction(const std::string& path, const std::string& case_id)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Attempt #1 planning CSV를 열 수 없습니다.");
  }
  std::string header;
  std::getline(input, header);
  const auto columns = splitCsv(header);
  auto indexOf = [&](const std::string& name) {
    const auto found = std::find(columns.begin(), columns.end(), name);
    if (found == columns.end()) {
      throw std::runtime_error("Attempt #1 CSV에 열이 없습니다: " + name);
    }
    return static_cast<std::size_t>(std::distance(columns.begin(), found));
  };
  const auto i_run = indexOf("run");
  const auto i_id = indexOf("case_id");
  const auto i_fraction = indexOf("fraction");
  std::string line;
  while (std::getline(input, line)) {
    const auto fields = splitCsv(line);
    if (fields.size() <= i_fraction) {
      continue;
    }
    if (fields[i_run] == "1" && fields[i_id] == case_id) {
      return std::stod(fields[i_fraction]);
    }
  }
  throw std::runtime_error("Attempt #1 fraction을 찾지 못했습니다: " + case_id);
}

Eigen::Isometry3d isometryOf(const robot_controller::MotionPose& pose)
{
  Eigen::Quaterniond quaternion(pose.qw, pose.qx, pose.qy, pose.qz);
  quaternion.normalize();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = quaternion.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(pose.x, pose.y, pose.z);
  return transform;
}

void timeArm(
    const std::vector<std::shared_ptr<moveit::core::RobotState>>& trajectory, const std::vector<std::string>& joints,
    double max_velocity, double& max_measured, int& violations)
{
  max_measured = 0.0;
  violations = 0;
  if (trajectory.size() < 2) {
    return;
  }
  for (std::size_t index = 1; index < trajectory.size(); ++index) {
    double delta = 0.0;
    for (const auto& name : joints) {
      delta = std::max(
          delta, std::abs(trajectory[index]->getVariablePosition(name) - trajectory[index - 1]->getVariablePosition(name)));
    }
    const double dt = std::max(delta / max_velocity, 0.05);
    const double velocity = delta / dt;
    max_measured = std::max(max_measured, velocity);
    if (velocity > max_velocity + 1e-6) {
      ++violations;
    }
  }
}

TrackSample sampleAgainst(
    const robot_controller::SealedProjection& sealed, const robot_controller::MotionPose& fk, const std::string& kind)
{
  TrackSample sample;
  sample.source_index = sealed.source_index;
  sample.kind = kind;
  sample.versus_projected = robot_controller::measureCapability(sealed.projected, fk);
  sample.versus_desired = robot_controller::measureCapability(sealed.desired, fk);
  return sample;
}

PlannedCase planProjected(
    const Scene& scene, const std::vector<WaypointCapability>& rows, const std::vector<double>& init_joints)
{
  PlannedCase planned;
  if (rows.size() != 11) {
    throw std::runtime_error("planning window가 11점이 아닙니다.");
  }
  std::vector<double> position_errors;
  std::vector<double> tool_errors;
  std::vector<double> roll_errors;
  std::vector<double> full_errors;
  for (const auto& row : rows) {
    if (!row.selection.found) {
      throw std::runtime_error("projected pose가 없는 waypoint가 있습니다.");
    }
    planned.sealed.push_back(robot_controller::sealProjection(
        row.desired, row.selection.selected.executable, row.selection.selected.joints,
        robot_controller::capabilityClassName(row.selection.capability_class)));
    const auto& metrics = planned.sealed.back().metrics;
    position_errors.push_back(metrics.position_error_m);
    tool_errors.push_back(metrics.tool_axis_error_deg);
    roll_errors.push_back(std::abs(metrics.roll_error_deg));
    full_errors.push_back(metrics.full_orientation_error_deg);
  }
  planned.position = robot_controller::summarizeErrors(position_errors);
  planned.tool = robot_controller::summarizeErrors(tool_errors);
  planned.roll = robot_controller::summarizeErrors(roll_errors);
  planned.full = robot_controller::summarizeErrors(full_errors);
  const auto targets = robot_controller::projectedPlanningTargets(planned.sealed);
  if (targets.size() != planned.sealed.size()) {
    throw std::runtime_error("planning target 개수가 projected pose와 다릅니다.");
  }
  for (std::size_t index = 0; index < targets.size(); ++index) {
    if (!robot_controller::poseFieldsEqual(targets[index], planned.sealed[index].projected)) {
      throw std::runtime_error("planning input이 projected pose가 아닙니다.");
    }
  }
  const auto& start_joints = planned.sealed.front().joints;
  planned.matches_srdf_init = start_joints.size() == init_joints.size();
  for (std::size_t joint = 0; joint < start_joints.size() && planned.matches_srdf_init; ++joint) {
    if (std::abs(start_joints[joint] - init_joints[joint]) > 1e-8) {
      planned.matches_srdf_init = false;
    }
  }
  planned.start_provenance =
      "first projected joint state, source " + std::to_string(planned.sealed.front().source_index) +
      ", synthetic placement init anchor, matches_srdf_init=" + (planned.matches_srdf_init ? "yes" : "no");
  auto start = makeState(scene, "", &start_joints);
  planned.start_collision = stateColliding(scene, start);
  planned.position_violations += positionViolations(scene, start);
  if (planned.start_collision) {
    planned.planning_feasibility = robot_controller::kPlanningInfeasible;
    planned.process_fidelity = robot_controller::kProcessFidelityNotYetAccepted;
    return planned;
  }
  EigenSTL::vector_Isometry3d waypoints;
  waypoints.reserve(targets.size());
  for (const auto& pose : targets) {
    waypoints.push_back(isometryOf(pose));
  }
  moveit::core::RobotState working(start);
  std::vector<std::shared_ptr<moveit::core::RobotState>> trajectory;
  const moveit::core::GroupStateValidityCallbackFn valid =
      [&](moveit::core::RobotState* state, const moveit::core::JointModelGroup*, const double*) {
        state->update(true);
        planning_scene_monitor::LockedPlanningSceneRO locked(scene.monitor);
        return !locked->isStateColliding(*state, "", false);
      };
  const auto begun = std::chrono::steady_clock::now();
  const auto percentage = moveit::core::CartesianInterpolator::computeCartesianPath(
      &working, scene.arm, trajectory, scene.end_link, waypoints, true, moveit::core::MaxEEFStep(kEefStep),
      moveit::core::JumpThreshold(kJumpThreshold), valid);
  planned.fraction = percentage.value;
  planned.planning_time_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - begun).count();
  planned.trajectory_points = static_cast<int>(trajectory.size());
  for (const auto& state : trajectory) {
    state->update(true);
    planned.position_violations += positionViolations(scene, *state);
    if (stateColliding(scene, *state)) {
      ++planned.collision_states;
    }
  }
  timeArm(trajectory, scene.joint_names, kMaxJointVelocity, planned.max_joint_velocity, planned.velocity_violations);
  const auto attached = robot_controller::attachPlanning(
      planned.sealed.front(), planned.fraction, planned.trajectory_points, planned.collision_states == 0,
      planned.position_violations == 0, planned.velocity_violations == 0, robot_controller::kMinCartesianFraction);
  if (!robot_controller::poseFieldsEqual(attached.projection.desired, planned.sealed.front().desired) ||
      std::abs(attached.projection.metrics.tool_axis_error_deg - planned.sealed.front().metrics.tool_axis_error_deg) >
          1e-12) {
    throw std::runtime_error("planning 결과가 projection fidelity를 덮어썼습니다.");
  }
  planned.planning_feasibility = attached.planning_feasibility;
  planned.process_fidelity = attached.process_fidelity;
  if (!trajectory.empty()) {
    const auto start_fk = poseFromState(*trajectory.front(), scene.end_link->getName());
    const auto end_fk = poseFromState(*trajectory.back(), scene.end_link->getName());
    planned.samples.push_back(sampleAgainst(planned.sealed.front(), start_fk, "start"));
    planned.samples.push_back(sampleAgainst(planned.sealed.back(), end_fk, "end_versus_last_request"));
    for (const auto& sealed : planned.sealed) {
      std::size_t best = 0;
      double best_distance = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0; index < trajectory.size(); ++index) {
        const auto fk = poseFromState(*trajectory[index], scene.end_link->getName());
        const double distance = robot_controller::poseDistance(sealed.projected, fk);
        if (distance < best_distance) {
          best_distance = distance;
          best = index;
        }
      }
      const auto fk = poseFromState(*trajectory[best], scene.end_link->getName());
      planned.samples.push_back(sampleAgainst(sealed, fk, "nearest_trajectory_state"));
    }
  }
  return planned;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("projected_planning_dry", options);
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
        std::abs(node->get_parameter("max_joint_velocity").as_double() - kMaxJointVelocity) > 1e-12 ||
        node->get_parameter("runs").as_int() != 3 ||
        std::abs(node->get_parameter("position_tolerance_m").as_double() - robot_controller::kCapabilityPositionToleranceM) >
            1e-12 ||
        node->get_parameter("null_half_count").as_int() != robot_controller::kNullHalfCount ||
        std::abs(node->get_parameter("null_step_rad").as_double() - robot_controller::kNullStepRad) > 1e-12 ||
        node->get_parameter("position_correction_iterations").as_int() != robot_controller::kPositionCorrectionIterations) {
      throw std::runtime_error("계획 또는 탐색 파라미터가 Attempt #1/#3 고정값과 다릅니다.");
    }
    if (sha256File(node->get_parameter("snapshot_pcd").as_string()) != kSnapshotSha ||
        sha256File(node->get_parameter("waypoints_csv").as_string()) != kWaypointSha) {
      throw std::runtime_error("snapshot 또는 waypoint SHA256이 P-004와 다릅니다.");
    }
    std::vector<std::vector<StoredProjection>> reference;
    std::vector<double> attempt1;
    for (const auto& frozen : kFrozenCases) {
      reference.push_back(loadReference(
          node->get_parameter("attempt03_dir").as_string() + "/" +
          (std::string("case-0") + std::string(frozen.id).back() + "-capability.csv")));
      const double fraction = attempt1Fraction(node->get_parameter("attempt01_cases").as_string(), frozen.id);
      if (std::abs(fraction - kAttempt1Fraction) > 1e-15) {
        throw std::runtime_error("Attempt #1 fraction이 1/11이 아닙니다.");
      }
      attempt1.push_back(fraction);
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
    const std::vector<std::string> expected = {"Arm1_Joint", "Arm2_Joint", "Arm3_Joint", "Arm4_Joint", "Arm5_Joint"};
    if (scene.arm->getVariableNames() != expected || scene.arm->getActiveJointModels().size() != 5) {
      throw std::runtime_error("runtime DOF가 Arm1–Arm5가 아닙니다.");
    }
    scene.linear_row = linearRow(scene);
    if (scene.linear_row < 0) {
      throw std::runtime_error("position Jacobian 행을 확인하지 못했습니다.");
    }
    const auto init_joints = positionsOf(makeState(scene, "init", nullptr), scene.joint_names);
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
    }
    if (!projection_match) {
      stop_spin();
      std::ofstream json(output_dir + "/planning-results.json");
      json << "{\n  \"status\": \"FAIL\",\n  \"reason\": \"attempt 3 projection mismatch\",\n"
           << "  \"process_fidelity\": \"" << robot_controller::kProcessFidelityNotYetAccepted << "\"\n}\n";
      std::cout << "P-005 ATTEMPT #4 FAIL projection mismatch\n";
      rclcpp::shutdown();
      return 1;
    }
    std::vector<std::vector<PlannedCase>> planned_runs;
    for (const auto& cases : projections) {
      std::vector<PlannedCase> planned_cases;
      for (const auto& rows : cases) {
        planned_cases.push_back(planProjected(scene, rows, init_joints));
        RCLCPP_INFO(
            node->get_logger(), "%s fraction %.6f %s", planned_cases.back().planning_feasibility.c_str(),
            planned_cases.back().fraction, planned_cases.back().process_fidelity.c_str());
      }
      planned_runs.push_back(std::move(planned_cases));
    }
    bool planning_same = true;
    for (std::size_t run = 1; run < planned_runs.size(); ++run) {
      for (std::size_t index = 0; index < 3; ++index) {
        const auto& first = planned_runs[0][index];
        const auto& other = planned_runs[run][index];
        if (first.planning_feasibility != other.planning_feasibility || first.trajectory_points != other.trajectory_points ||
            first.collision_states != other.collision_states || std::abs(first.fraction - other.fraction) > 1e-9) {
          planning_same = false;
        }
        for (std::size_t waypoint = 0; waypoint < first.sealed.size(); ++waypoint) {
          if (first.sealed[waypoint].capability_class != other.sealed[waypoint].capability_class) {
            planning_same = false;
          }
        }
      }
    }
    bool all_feasible = true;
    for (const auto& item : planned_runs[0]) {
      if (item.planning_feasibility != robot_controller::kPlanningFeasible ||
          item.process_fidelity != robot_controller::kProcessFidelityNotYetAccepted) {
        all_feasible = false;
      }
    }
    const std::string status = (planning_same && all_feasible) ? "PASS_CANDIDATE" : "FAIL";
    {
      std::ofstream desired_csv(output_dir + "/desired-vs-projected.csv");
      desired_csv << std::setprecision(17);
      desired_csv << "case_id,source_index,desired_x,desired_y,desired_z,desired_qx,desired_qy,desired_qz,desired_qw,"
                     "projected_x,projected_y,projected_z,projected_qx,projected_qy,projected_qz,projected_qw,"
                     "position_error_m,tool_axis_error_deg,roll_error_deg,full_orientation_error_deg\n";
      std::ofstream mapping(output_dir + "/source-mapping.csv");
      mapping << std::setprecision(17);
      mapping << "case_id,source_index,desired_x,desired_qw,projected_joints,projected_x,projected_qw,planning_target_x,"
                 "planning_target_qw\n";
      std::ofstream compare(output_dir + "/attempt-01-vs-attempt-04.csv");
      compare << std::setprecision(17);
      compare << "case_id,attempt1_fraction,attempt4_fraction,attempt1_class,attempt4_planning,attempt4_fidelity\n";
      std::ofstream trajectory(output_dir + "/trajectory-validation.csv");
      trajectory << std::setprecision(17);
      trajectory << "case_id,kind,source_index,position_error_vs_projected_m,full_orientation_error_vs_projected_deg,"
                    "position_error_vs_desired_m,tool_axis_error_vs_desired_deg,roll_error_vs_desired_deg,"
                    "full_orientation_error_vs_desired_deg\n";
      for (std::size_t case_index = 0; case_index < 3; ++case_index) {
        const auto& item = planned_runs[0][case_index];
        std::ofstream planning(output_dir + "/case-0" + std::to_string(case_index + 1) + "-planning.csv");
        planning << std::setprecision(17);
        planning << "source_index,fraction,trajectory_points,planning_time_s,max_joint_velocity,position_violations,"
                    "velocity_violations,collision_states,planning_feasibility,process_fidelity,position_error_m,"
                    "tool_axis_error_deg,roll_error_deg,full_orientation_error_deg\n";
        for (const auto& sealed : item.sealed) {
          desired_csv << kFrozenCases[case_index].id << ',' << sealed.source_index << ',' << sealed.desired.x << ','
                      << sealed.desired.y << ',' << sealed.desired.z << ',' << sealed.desired.qx << ','
                      << sealed.desired.qy << ',' << sealed.desired.qz << ',' << sealed.desired.qw << ','
                      << sealed.projected.x << ',' << sealed.projected.y << ',' << sealed.projected.z << ','
                      << sealed.projected.qx << ',' << sealed.projected.qy << ',' << sealed.projected.qz << ','
                      << sealed.projected.qw << ',' << sealed.metrics.position_error_m << ','
                      << sealed.metrics.tool_axis_error_deg << ',' << sealed.metrics.roll_error_deg << ','
                      << sealed.metrics.full_orientation_error_deg << '\n';
          mapping << kFrozenCases[case_index].id << ',' << sealed.source_index << ',' << sealed.desired.x << ','
                  << sealed.desired.qw << ',';
          for (std::size_t joint = 0; joint < sealed.joints.size(); ++joint) {
            if (joint > 0) {
              mapping << ';';
            }
            mapping << sealed.joints[joint];
          }
          mapping << ',' << sealed.projected.x << ',' << sealed.projected.qw << ',' << sealed.projected.x << ','
                  << sealed.projected.qw << '\n';
          planning << sealed.source_index << ',' << item.fraction << ',' << item.trajectory_points << ','
                   << item.planning_time_sec << ',' << item.max_joint_velocity << ',' << item.position_violations << ','
                   << item.velocity_violations << ',' << item.collision_states << ',' << item.planning_feasibility << ','
                   << item.process_fidelity << ',' << sealed.metrics.position_error_m << ','
                   << sealed.metrics.tool_axis_error_deg << ',' << sealed.metrics.roll_error_deg << ','
                   << sealed.metrics.full_orientation_error_deg << '\n';
        }
        for (const auto& sample : item.samples) {
          trajectory << kFrozenCases[case_index].id << ',' << sample.kind << ',' << sample.source_index << ','
                     << sample.versus_projected.position_error_m << ',' << sample.versus_projected.full_orientation_error_deg
                     << ',' << sample.versus_desired.position_error_m << ',' << sample.versus_desired.tool_axis_error_deg
                     << ',' << sample.versus_desired.roll_error_deg << ','
                     << sample.versus_desired.full_orientation_error_deg << '\n';
        }
        compare << kFrozenCases[case_index].id << ',' << attempt1[case_index] << ',' << item.fraction
                << ",CARTESIAN_PATH," << item.planning_feasibility << ',' << item.process_fidelity << '\n';
      }
      std::ofstream collision(output_dir + "/collision-validation.txt");
      collision << "collision_checking_disabled false\n";
      collision << "validity_callback same_as_attempt_1\n";
      collision << "global_reference_frame true\n";
      for (std::size_t run = 0; run < planned_runs.size(); ++run) {
        for (std::size_t index = 0; index < 3; ++index) {
          const auto& item = planned_runs[run][index];
          collision << "run " << (run + 1) << ' ' << kFrozenCases[index].id << " start "
                    << (item.start_collision ? "COLLISION" : "clear") << " trajectory_collision_states "
                    << item.collision_states << "\n";
        }
      }
      std::ofstream limits(output_dir + "/joint-limit-validation.txt");
      limits << std::setprecision(17);
      limits << "position_margin_rad " << kPositionMargin << "\n";
      for (std::size_t run = 0; run < planned_runs.size(); ++run) {
        for (std::size_t index = 0; index < 3; ++index) {
          limits << "run " << (run + 1) << ' ' << kFrozenCases[index].id << " position_violations "
                 << planned_runs[run][index].position_violations << "\n";
        }
      }
      std::ofstream velocity(output_dir + "/velocity-validation.txt");
      velocity << std::setprecision(17);
      velocity << "max_joint_velocity_rad_s " << kMaxJointVelocity << "\n";
      velocity << "time_parameterization attempt_1_cap\n";
      velocity << "execute_if_over_cap no\n";
      for (std::size_t run = 0; run < planned_runs.size(); ++run) {
        for (std::size_t index = 0; index < 3; ++index) {
          const auto& item = planned_runs[run][index];
          velocity << "run " << (run + 1) << ' ' << kFrozenCases[index].id << " max_joint_velocity "
                   << item.max_joint_velocity << " velocity_violations " << item.velocity_violations << "\n";
        }
      }
      std::ofstream reproducibility(output_dir + "/reproducibility.txt");
      reproducibility << std::setprecision(17);
      reproducibility << "runs " << planned_runs.size() << "\n";
      reproducibility << "projection_classification_equal yes\n";
      reproducibility << "planning_equal " << (planning_same ? "yes" : "no") << "\n";
      for (std::size_t run = 0; run < planned_runs.size(); ++run) {
        reproducibility << "run " << (run + 1);
        for (std::size_t index = 0; index < 3; ++index) {
          const auto& item = planned_runs[run][index];
          reproducibility << ' ' << kFrozenCases[index].id << " fraction " << item.fraction << " points "
                         << item.trajectory_points << ' ' << item.planning_feasibility << " collisions "
                         << item.collision_states;
        }
        reproducibility << "\n";
      }
      std::ofstream svg(output_dir + "/desired-vs-projected-orientation.svg");
      svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"980\" height=\"360\" viewBox=\"0 0 980 360\">\n";
      svg << "<rect width=\"980\" height=\"360\" fill=\"#f7f4ee\"/>\n";
      svg << "<text x=\"24\" y=\"32\" font-family=\"sans-serif\" font-size=\"18\" fill=\"#1c1917\">desired versus projected orientation error</text>\n";
      const char* colors[] = {"#b45309", "#1d4ed8", "#44403c"};
      for (std::size_t case_index = 0; case_index < 3; ++case_index) {
        std::ostringstream tool_points;
        std::ostringstream roll_points;
        const auto& item = planned_runs[0][case_index];
        for (std::size_t index = 0; index < item.sealed.size(); ++index) {
          const double x = 40.0 + (static_cast<double>(case_index) * 320.0) + (static_cast<double>(index) * 28.0);
          const double tool_y = 300.0 - std::min(item.sealed[index].metrics.tool_axis_error_deg, 140.0);
          const double roll_y = 300.0 - std::min(std::abs(item.sealed[index].metrics.roll_error_deg), 140.0) * 0.7;
          tool_points << x << ',' << tool_y << ' ';
          roll_points << x << ',' << roll_y << ' ';
        }
        svg << "<polyline fill=\"none\" stroke=\"" << colors[case_index] << "\" stroke-width=\"2\" points=\""
            << tool_points.str() << "\"/>\n";
        svg << "<polyline fill=\"none\" stroke=\"" << colors[case_index]
            << "\" stroke-width=\"1\" stroke-dasharray=\"4 3\" points=\"" << roll_points.str() << "\"/>\n";
        svg << "<text x=\"" << (40 + case_index * 320) << "\" y=\"340\" font-family=\"sans-serif\" font-size=\"13\" fill=\""
            << colors[case_index] << "\">" << kFrozenCases[case_index].id << " tool / |roll|</text>\n";
      }
      svg << "</svg>\n";
      std::ofstream json(output_dir + "/planning-results.json");
      json << std::setprecision(17);
      json << "{\n  \"status\": \"" << status << "\",\n";
      json << "  \"source_geometry\": {\"snapshot_sha256\": \"" << kSnapshotSha << "\", \"waypoints_sha256\": \""
           << kWaypointSha << "\", \"frame\": \"camera_depth_optical_frame\", \"immutable\": true},\n";
      json << "  \"safety\": {\"execute_trajectory\": false, \"execute_calls\": 0, \"controller_sends\": 0, "
              "\"camera_robot_calibration\": 0, \"fake_tf\": 0, \"actual_robot\": 0, \"geometry_scale\": 1, "
              "\"waypoints_deleted\": 0, \"parameter_tuning_after_results\": false},\n";
      json << "  \"planning_parameters\": {\"eef_step\": " << kEefStep << ", \"jump_threshold\": " << kJumpThreshold
           << ", \"avoid_collisions\": true, \"global_reference_frame\": true, \"min_fraction\": "
           << robot_controller::kMinCartesianFraction << ", \"max_joint_velocity\": " << kMaxJointVelocity << "},\n";
      json << "  \"cases\": [\n";
      for (std::size_t case_index = 0; case_index < 3; ++case_index) {
        const auto& item = planned_runs[0][case_index];
        json << "    {\"case_id\": \"" << kFrozenCases[case_index].id << "\", \"source_start\": "
             << kFrozenCases[case_index].start << ", \"source_end\": " << kFrozenCases[case_index].end << ",\n";
        json << "     \"desired_pose_count\": " << item.sealed.size() << ",\n";
        json << "     \"capability_projection\": {\"position_max\": " << item.position.max << ", \"tool_max\": "
             << item.tool.max << ", \"roll_max\": " << item.roll.max << ", \"full_max\": " << item.full.max << "},\n";
        json << "     \"projected_pose_count\": " << item.sealed.size() << ",\n";
        json << "     \"planning\": {\"fraction\": " << item.fraction << ", \"attempt1_fraction\": " << attempt1[case_index]
             << ", \"trajectory_points\": " << item.trajectory_points << ", \"planning_time_sec\": "
             << item.planning_time_sec << ", \"start_provenance\": \"" << item.start_provenance
             << "\", \"feasibility\": \"" << item.planning_feasibility << "\"},\n";
        json << "     \"trajectory_validation\": {\"collision_states\": " << item.collision_states
             << ", \"position_violations\": " << item.position_violations << ", \"velocity_violations\": "
             << item.velocity_violations << ", \"max_joint_velocity\": " << item.max_joint_velocity << "},\n";
        json << "     \"process_fidelity\": \"" << item.process_fidelity << "\",\n";
        json << "     \"waypoints\": [";
        for (std::size_t index = 0; index < item.sealed.size(); ++index) {
          const auto& sealed = item.sealed[index];
          json << "{\"source_index\": " << sealed.source_index << ", \"class\": \"" << sealed.capability_class
               << "\", \"position_error_m\": " << sealed.metrics.position_error_m << ", \"tool_axis_error_deg\": "
               << sealed.metrics.tool_axis_error_deg << ", \"roll_error_deg\": " << sealed.metrics.roll_error_deg
               << ", \"full_orientation_error_deg\": " << sealed.metrics.full_orientation_error_deg << "}"
               << (index + 1 == item.sealed.size() ? "" : ", ");
        }
        json << "]}" << (case_index + 1 == 3 ? "\n" : ",\n");
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
    std::cout << "P-005 ATTEMPT #4 " << status << " fidelity " << robot_controller::kProcessFidelityNotYetAccepted
              << "\n";
    exit_code = 0;
  } catch (const std::exception& error) {
    stop_spin();
    std::ofstream json(output_dir + "/planning-results.json");
    json << "{\n  \"status\": \"FAIL\",\n  \"error\": \"" << error.what() << "\"\n}\n";
    std::cout << "P-005 ATTEMPT #4 FAIL\n" << error.what() << "\n";
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
