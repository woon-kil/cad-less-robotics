// Copyright 2026 cad-less-robotics contributors
// P-005 attempt 3. Desired pose를 덮어쓰지 않고, DOFBOT이 그 위치에서 만들 수 있는 자세만 측정한다.
// computeCartesianPath에는 넣지 않는다.

#include "robot_controller/capability_adapter.hpp"
#include "robot_controller/motion_contract.hpp"
#include "vision_path_generator/polyline.hpp"

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

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

std::string num(double value)
{
  std::ostringstream output;
  output << std::setprecision(17) << value;
  return output.str();
}

std::string flag(bool value)
{
  return value ? "true" : "false";
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
    if (values[index] < bounds.min_position_ - 1e-6 || values[index] > bounds.max_position_ + 1e-6) {
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
      candidates.push_back(
          makeCandidate(scene, desired, positioned, robot_controller::CandidateOrigin::PositionSeed));
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
      results[index].desired_change = robot_controller::measureCapability(results[index - 1].desired, results[index].desired);
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
  const std::size_t index = 1;
  const double delta = 1e-6;
  auto shifted = seed;
  shifted[index] += delta;
  auto perturbed = makeState(scene, "", &shifted);
  const Eigen::Vector3d difference = perturbed.getGlobalLinkTransform(scene.end_link).translation() -
                                     state.getGlobalLinkTransform(scene.end_link).translation();
  const double scale = std::max(difference.norm(), 1e-12);
  const double top = (jacobian.block(0, static_cast<Eigen::Index>(index), 3, 1) * delta - difference).norm() / scale;
  const double bottom =
      (jacobian.block(3, static_cast<Eigen::Index>(index), 3, 1) * delta - difference).norm() / scale;
  if (top <= bottom && top < 0.05) {
    return 0;
  }
  if (bottom < 0.05) {
    return 3;
  }
  return -1;
}

void writeSvg(const std::string& path, const std::string& title, const std::vector<WaypointCapability>& rows)
{
  double max_error = 10.0;
  for (const auto& row : rows) {
    if (!row.selection.found) {
      continue;
    }
    max_error = std::max(max_error, row.selection.selected.metrics.tool_axis_error_deg);
    max_error = std::max(max_error, std::abs(row.selection.selected.metrics.roll_error_deg));
    max_error = std::max(max_error, row.selection.selected.metrics.full_orientation_error_deg);
  }
  std::ofstream output(path);
  output << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"980\" height=\"420\" viewBox=\"0 0 980 420\">\n";
  output << "<rect width=\"980\" height=\"420\" fill=\"#f7f4ee\"/>\n";
  output << "<text x=\"24\" y=\"32\" font-family=\"sans-serif\" font-size=\"18\" fill=\"#1c1917\">" << title
         << "</text>\n";
  auto polyline = [&](const char* color, int mode) {
    std::ostringstream points;
    for (std::size_t index = 0; index < rows.size(); ++index) {
      if (!rows[index].selection.found) {
        continue;
      }
      const auto& metrics = rows[index].selection.selected.metrics;
      const double value = mode == 0 ? metrics.tool_axis_error_deg :
                           mode == 1 ? std::abs(metrics.roll_error_deg) :
                                       metrics.full_orientation_error_deg;
      const double x = 50.0 + (static_cast<double>(index) * 520.0 / 10.0);
      const double y = 300.0 - (value / max_error * 230.0);
      points << x << "," << y << " ";
    }
    output << "<polyline fill=\"none\" stroke=\"" << color << "\" stroke-width=\"2\" points=\"" << points.str()
           << "\"/>\n";
  };
  output << "<line x1=\"50\" y1=\"300\" x2=\"570\" y2=\"300\" stroke=\"#44403c\"/>\n";
  output << "<line x1=\"50\" y1=\"70\" x2=\"50\" y2=\"300\" stroke=\"#44403c\"/>\n";
  polyline("#b45309", 0);
  polyline("#1d4ed8", 1);
  polyline("#44403c", 2);
  output << "<text x=\"50\" y=\"340\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#b45309\">tool axis</text>\n";
  output << "<text x=\"150\" y=\"340\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#1d4ed8\">|roll|</text>\n";
  output << "<text x=\"230\" y=\"340\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#44403c\">full orientation</text>\n";
  output << "<text x=\"620\" y=\"88\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#1c1917\">executable tool axis in desired frame</text>\n";
  output << "<circle cx=\"790\" cy=\"210\" r=\"110\" fill=\"none\" stroke=\"#d6d3d1\"/>\n";
  output << "<circle cx=\"790\" cy=\"210\" r=\"3\" fill=\"#1c1917\"/>\n";
  for (const auto& row : rows) {
    if (!row.selection.found) {
      continue;
    }
    const auto metrics_pose = row.selection.selected.executable;
    Eigen::Quaterniond desired(row.desired.qw, row.desired.qx, row.desired.qy, row.desired.qz);
    Eigen::Quaterniond executable(metrics_pose.qw, metrics_pose.qx, metrics_pose.qy, metrics_pose.qz);
    desired.normalize();
    executable.normalize();
    const Eigen::Vector3d local = desired.conjugate() * (executable * Eigen::Vector3d::UnitZ());
    const double x = 790.0 + (local.x() * 110.0);
    const double y = 210.0 - (local.y() * 110.0);
    output << "<circle cx=\"" << x << "\" cy=\"" << y << "\" r=\"4\" fill=\"#b45309\"/>\n";
  }
  output << "</svg>\n";
}

bool sameRun(const std::vector<WaypointCapability>& lhs, const std::vector<WaypointCapability>& rhs, double& joint_diff, double& metric_diff)
{
  joint_diff = 0.0;
  metric_diff = 0.0;
  if (lhs.size() != rhs.size()) {
    return false;
  }
  bool same = true;
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    if (lhs[index].selection.capability_class != rhs[index].selection.capability_class ||
        lhs[index].selection.found != rhs[index].selection.found) {
      same = false;
    }
    if (!lhs[index].selection.found || !rhs[index].selection.found) {
      continue;
    }
    const auto& a = lhs[index].selection.selected;
    const auto& b = rhs[index].selection.selected;
    metric_diff = std::max(metric_diff, std::abs(a.metrics.position_error_m - b.metrics.position_error_m));
    metric_diff = std::max(metric_diff, std::abs(a.metrics.tool_axis_error_deg - b.metrics.tool_axis_error_deg));
    metric_diff = std::max(metric_diff, std::abs(a.metrics.roll_error_deg - b.metrics.roll_error_deg));
    metric_diff = std::max(metric_diff, std::abs(a.metrics.full_orientation_error_deg - b.metrics.full_orientation_error_deg));
    if (a.joints.size() == b.joints.size()) {
      for (std::size_t joint = 0; joint < a.joints.size(); ++joint) {
        joint_diff = std::max(joint_diff, std::abs(a.joints[joint] - b.joints[joint]));
      }
    } else {
      same = false;
    }
  }
  if (joint_diff > 1e-8 || metric_diff > 1e-6) {
    same = false;
  }
  return same;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("capability_adapter_dry", options);
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
    if (std::abs(node->get_parameter("position_tolerance_m").as_double() - robot_controller::kCapabilityPositionToleranceM) > 1e-12 ||
        node->get_parameter("null_half_count").as_int() != robot_controller::kNullHalfCount ||
        std::abs(node->get_parameter("null_step_rad").as_double() - robot_controller::kNullStepRad) > 1e-12 ||
        node->get_parameter("position_correction_iterations").as_int() != robot_controller::kPositionCorrectionIterations ||
        std::abs(node->get_parameter("exact_orientation_deg").as_double() - robot_controller::kExactOrientationDeg) > 1e-12 ||
        std::abs(node->get_parameter("branch_discontinuity_rad").as_double() - robot_controller::kBranchDiscontinuityRad) > 1e-12 ||
        node->get_parameter("capability_runs").as_int() != robot_controller::kCapabilityRuns) {
      throw std::runtime_error("탐색 설정이 코드에 고정한 값과 다릅니다.");
    }
    if (sha256File(node->get_parameter("snapshot_pcd").as_string()) != kSnapshotSha ||
        sha256File(node->get_parameter("waypoints_csv").as_string()) != kWaypointSha) {
      throw std::runtime_error("snapshot 또는 waypoint SHA256이 P-004와 다릅니다.");
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
      if (worst >= 1e-9) {
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
      placed_cases.push_back(std::move(placed));
    }
    for (std::size_t index = 0; index < camera_poses.size(); ++index) {
      if (camera_poses[index].x != camera_copy[index].x || camera_poses[index].qx != camera_copy[index].qx ||
          camera_poses[index].qw != camera_copy[index].qw || camera_poses[index].nx != camera_copy[index].nx) {
        throw std::runtime_error("desired geometry가 바뀌었습니다.");
      }
    }
    std::vector<std::vector<std::vector<WaypointCapability>>> runs;
    for (int run = 0; run < robot_controller::kCapabilityRuns; ++run) {
      std::vector<std::vector<WaypointCapability>> cases;
      for (const auto& placed : placed_cases) {
        cases.push_back(searchCase(scene, placed));
        RCLCPP_INFO(node->get_logger(), "run %d case done, last class %s", run + 1,
                    robot_controller::capabilityClassName(cases.back().back().selection.capability_class));
      }
      runs.push_back(std::move(cases));
    }
    double joint_diff = 0.0;
    double metric_diff = 0.0;
    bool reproducible = true;
    for (int run = 1; run < robot_controller::kCapabilityRuns; ++run) {
      for (std::size_t case_index = 0; case_index < 3; ++case_index) {
        double joints = 0.0;
        double metrics = 0.0;
        if (!sameRun(runs[0][case_index], runs[run][case_index], joints, metrics)) {
          reproducible = false;
        }
        joint_diff = std::max(joint_diff, joints);
        metric_diff = std::max(metric_diff, metrics);
      }
    }
    {
      std::ofstream design(output_dir + "/adapter-design.txt");
      design << "DesiredGeometryPose는 P-004 position, quaternion, normal이다. 이 노드는 그 값을 쓰지 않고 읽기만 한다.\n";
      design << "ExecutableRobotPose는 별도 후보다. source index, desired pose, executable joint/FK, position/tool/roll/full error, class를 가진다.\n";
      design << "탐색은 위치 Jacobian의 영공간 격자다. 5관절에서 위치 제약의 남은 자유도는 2다.\n";
      design << "격자 위에서 위치를 damped least squares로 0.5 mm 안에 되돌린 뒤 FK한다.\n";
      design << "seed는 SRDF init, up, down이다. full-pose KDL은 timeout 0의 한 번 시도만 쓰고 난수 재시작을 쓰지 않는다.\n";
      design << "후보 비교는 position tolerance, tool-axis error, full orientation error 순이다. 공정 합격 각도는 정하지 않았다.\n";
      design << "1 deg 이내 full orientation은 EXACT_POSE라는 수치 일치 라벨일 뿐 공정 합격이 아니다.\n";
      design << "null-space reference: position-only Jacobian null motion changes orientation.\n";
      design << "https://robotics.stackexchange.com/questions/23600/redundancy-and-null-space-projection\n";
      design << "https://www.engr.colostate.edu/~aam/pdf/journals/104.pdf\n";
      design << "computeCartesianPath 호출 0\nproduction motion contract 변경 0\n";
    }
    {
      std::ofstream config(output_dir + "/capability-search-config.yaml");
      config << std::setprecision(17);
      config << "position_tolerance_m: " << robot_controller::kCapabilityPositionToleranceM << "\n";
      config << "null_half_count: " << robot_controller::kNullHalfCount << "\n";
      config << "null_step_rad: " << robot_controller::kNullStepRad << "\n";
      config << "position_correction_iterations: " << robot_controller::kPositionCorrectionIterations << "\n";
      config << "exact_orientation_deg: " << robot_controller::kExactOrientationDeg << "\n";
      config << "branch_discontinuity_rad: " << robot_controller::kBranchDiscontinuityRad << "\n";
      config << "capability_runs: " << robot_controller::kCapabilityRuns << "\n";
      config << "jacobian_linear_row: " << scene.linear_row << "\n";
      config << "seeds: [init, up, down]\n";
      config << "widened_after_results: false\n";
    }
    std::ofstream orientation(output_dir + "/orientation-projection.csv");
    orientation << "case_id,source_index,position_error_m,tool_axis_error_deg,roll_error_deg,full_orientation_error_deg,"
                   "desired_tool_step_deg,desired_roll_step_deg,desired_full_step_deg,capability_class\n";
    orientation << std::setprecision(17);
    std::ofstream continuity(output_dir + "/joint-continuity.csv");
    continuity << "case_id,from_index,to_index,dq1,dq2,dq3,dq4,dq5,max_abs_rad,branch_discontinuity\n";
    continuity << std::setprecision(17);
    std::ofstream collisions(output_dir + "/collision-analysis.csv");
    collisions << "case_id,source_index,capability_class,selected_colliding,collision_rejected,limit_rejected\n";
    std::ofstream limits(output_dir + "/joint-limit-analysis.csv");
    limits << "case_id,source_index,joint,value,lower,upper,margin_to_lower,margin_to_upper,violates\n";
    limits << std::setprecision(17);
    std::ofstream summary(output_dir + "/capability-summary.json");
    summary << std::setprecision(17);
    summary << "{\n  \"status\": \"" << (reproducible ? "DIAGNOSIS_COMPLETE" : "INCOMPLETE") << "\",\n  \"cases\": [\n";
    for (std::size_t case_index = 0; case_index < 3; ++case_index) {
      const auto& rows = runs[0][case_index];
      std::ofstream csv(output_dir + "/case-0" + std::to_string(case_index + 1) + "-capability.csv");
      csv << std::setprecision(17);
      csv << "source_index,desired_x,desired_y,desired_z,desired_qx,desired_qy,desired_qz,desired_qw,";
      csv << "executable_x,executable_y,executable_z,executable_qx,executable_qy,executable_qz,executable_qw,";
      csv << "position_error_m,tool_axis_error_deg,roll_error_deg,full_orientation_error_deg,joints,";
      csv << "capability_class,branch_discontinuity,max_abs_joint_step_rad,within_tolerance,null_grid_within_tolerance,";
      csv << "collision_rejected,limit_rejected\n";
      std::vector<double> position_errors;
      std::vector<double> tool_errors;
      std::vector<double> roll_errors;
      std::vector<double> full_errors;
      std::vector<double> steps;
      std::array<double, 5> joint_max{};
      int counts[6] = {};
      int branches = 0;
      int limit_failures = 0;
      int collision_rejects = 0;
      for (std::size_t index = 0; index < rows.size(); ++index) {
        const auto& row = rows[index];
        const auto& selected = row.selection.selected;
        const bool found = row.selection.found;
        csv << row.source_index << "," << row.desired.x << "," << row.desired.y << "," << row.desired.z << ","
            << row.desired.qx << "," << row.desired.qy << "," << row.desired.qz << "," << row.desired.qw << ",";
        if (found) {
          csv << selected.executable.x << "," << selected.executable.y << "," << selected.executable.z << ","
              << selected.executable.qx << "," << selected.executable.qy << "," << selected.executable.qz << ","
              << selected.executable.qw << "," << selected.metrics.position_error_m << ","
              << selected.metrics.tool_axis_error_deg << "," << selected.metrics.roll_error_deg << ","
              << selected.metrics.full_orientation_error_deg << ",";
          for (std::size_t joint = 0; joint < selected.joints.size(); ++joint) {
            if (joint > 0) {
              csv << ";";
            }
            csv << selected.joints[joint];
          }
          position_errors.push_back(selected.metrics.position_error_m);
          tool_errors.push_back(selected.metrics.tool_axis_error_deg);
          roll_errors.push_back(std::abs(selected.metrics.roll_error_deg));
          full_errors.push_back(selected.metrics.full_orientation_error_deg);
        } else {
          csv << ",,,,,,,,,,,";
        }
        csv << "," << robot_controller::capabilityClassName(row.selection.capability_class) << ","
            << flag(row.step.branch_discontinuity) << "," << row.step.max_abs << "," << row.selection.within_tolerance
            << "," << row.selection.null_grid_within_tolerance << "," << row.selection.collision_rejected << ","
            << row.selection.limit_rejected << "\n";
        orientation << kFrozenCases[case_index].id << "," << row.source_index << ",";
        if (found) {
          orientation << selected.metrics.position_error_m << "," << selected.metrics.tool_axis_error_deg << ","
                      << selected.metrics.roll_error_deg << "," << selected.metrics.full_orientation_error_deg << ",";
        } else {
          orientation << ",,,";
        }
        orientation << row.desired_change.tool_axis_error_deg << "," << row.desired_change.roll_error_deg << ","
                    << row.desired_change.full_orientation_error_deg << ","
                    << robot_controller::capabilityClassName(row.selection.capability_class) << "\n";
        if (index > 0) {
          continuity << kFrozenCases[case_index].id << "," << rows[index - 1].source_index << "," << row.source_index;
          for (std::size_t joint = 0; joint < 5; ++joint) {
            continuity << "," << (joint < row.step.delta.size() ? num(row.step.delta[joint]) : "");
          }
          continuity << "," << row.step.max_abs << "," << flag(row.step.branch_discontinuity) << "\n";
        }
        collisions << kFrozenCases[case_index].id << "," << row.source_index << ","
                   << robot_controller::capabilityClassName(row.selection.capability_class) << ","
                   << flag(found && selected.colliding) << "," << row.selection.collision_rejected << ","
                   << row.selection.limit_rejected << "\n";
        if (found) {
          for (std::size_t joint = 0; joint < selected.joints.size(); ++joint) {
            const auto& bounds = scene.model->getVariableBounds(scene.joint_names[joint]);
            const double lower = bounds.min_position_;
            const double upper = bounds.max_position_;
            const bool violates = selected.joints[joint] < lower - 1e-6 || selected.joints[joint] > upper + 1e-6;
            if (violates) {
              ++limit_failures;
            }
            limits << kFrozenCases[case_index].id << "," << row.source_index << "," << scene.joint_names[joint] << ","
                   << selected.joints[joint] << "," << lower << "," << upper << "," << (selected.joints[joint] - lower)
                   << "," << (upper - selected.joints[joint]) << "," << flag(violates) << "\n";
            if (joint < joint_max.size()) {
              joint_max[joint] = std::max(joint_max[joint], std::abs(joint < row.step.delta.size() ? row.step.delta[joint] : 0.0));
            }
          }
        } else {
          limits << kFrozenCases[case_index].id << "," << row.source_index << ",no_solution,,,,,,\n";
        }
        counts[static_cast<int>(row.selection.capability_class)] += 1;
        if (row.step.branch_discontinuity) {
          ++branches;
        }
        if (!row.step.delta.empty()) {
          steps.push_back(row.step.max_abs);
        }
        collision_rejects += row.selection.collision_rejected;
      }
      const auto position = robot_controller::summarizeErrors(position_errors);
      const auto tool = robot_controller::summarizeErrors(tool_errors);
      const auto roll = robot_controller::summarizeErrors(roll_errors);
      const auto full = robot_controller::summarizeErrors(full_errors);
      const auto step_summary = robot_controller::summarizeErrors(steps);
      summary << "    {\"case_id\": \"" << kFrozenCases[case_index].id << "\", \"waypoints\": " << rows.size()
              << ", \"exact\": " << counts[0] << ", \"near\": " << counts[1] << ", \"position_only\": " << counts[2]
              << ", \"no_position\": " << counts[3] << ", \"collision\": " << counts[4] << ", \"joint_limit\": " << counts[5]
              << ", \"branch_discontinuities\": " << branches << ", \"collision_rejected\": " << collision_rejects
              << ", \"selected_limit_violations\": " << limit_failures
              << ", \"position_mean\": " << position.mean << ", \"position_p95\": " << position.p95
              << ", \"position_max\": " << position.max << ", \"tool_mean\": " << tool.mean << ", \"tool_p95\": " << tool.p95
              << ", \"tool_max\": " << tool.max << ", \"roll_mean\": " << roll.mean << ", \"roll_p95\": " << roll.p95
              << ", \"roll_max\": " << roll.max << ", \"full_mean\": " << full.mean << ", \"full_p95\": " << full.p95
              << ", \"full_max\": " << full.max << ", \"joint_step_mean\": " << step_summary.mean
              << ", \"joint_step_max\": " << step_summary.max << ", \"joint_max_abs\": [" << joint_max[0] << ", "
              << joint_max[1] << ", " << joint_max[2] << ", " << joint_max[3] << ", " << joint_max[4] << "], \"waypoints_detail\": [";
      for (std::size_t index = 0; index < rows.size(); ++index) {
        const auto& row = rows[index];
        summary << "{\"source_index\": " << row.source_index << ", \"class\": \""
                << robot_controller::capabilityClassName(row.selection.capability_class) << "\"";
        if (row.selection.found) {
          summary << ", \"position_error_m\": " << row.selection.selected.metrics.position_error_m
                  << ", \"tool_axis_error_deg\": " << row.selection.selected.metrics.tool_axis_error_deg
                  << ", \"roll_error_deg\": " << row.selection.selected.metrics.roll_error_deg
                  << ", \"full_orientation_error_deg\": " << row.selection.selected.metrics.full_orientation_error_deg
                  << ", \"desired_tool_step_deg\": " << row.desired_change.tool_axis_error_deg
                  << ", \"desired_roll_step_deg\": " << row.desired_change.roll_error_deg
                  << ", \"desired_full_step_deg\": " << row.desired_change.full_orientation_error_deg;
        }
        summary << "}" << (index + 1 == rows.size() ? "" : ", ");
      }
      summary << "]}" << (case_index + 1 == 3 ? "\n" : ",\n");
      writeSvg(output_dir + "/case-0" + std::to_string(case_index + 1) + "-orientation-projection.svg",
               std::string(kFrozenCases[case_index].id) + " orientation projection", rows);
    }
    summary << "  ],\n  \"reproducible\": " << (reproducible ? "true" : "false") << ", \"max_joint_diff_rad\": " << joint_diff
            << ", \"max_metric_diff\": " << metric_diff
            << ",\n  \"production_motion_contract_changed\": false,\n  \"p004_changed\": false,\n"
            << "  \"compute_cartesian_path_calls\": 0,\n  \"camera_robot_calibration\": 0,\n  \"fake_tf\": 0,\n"
            << "  \"execute_calls\": 0,\n  \"controller_sends\": 0\n}\n";
    std::ofstream reproducibility(output_dir + "/reproducibility.txt");
    reproducibility << std::setprecision(17);
    reproducibility << "runs " << robot_controller::kCapabilityRuns << "\n";
    reproducibility << "same_classification " << (reproducible ? "yes" : "no") << "\n";
    reproducibility << "max_joint_diff_rad " << joint_diff << "\n";
    reproducibility << "max_metric_diff " << metric_diff << "\n";
    reproducibility << "joint_tolerance_rad 1e-8\nmetric_tolerance 1e-6\n";
    stop_spin();
    for (std::size_t index = 0; index < camera_poses.size(); ++index) {
      if (camera_poses[index].x != camera_copy[index].x || camera_poses[index].qw != camera_copy[index].qw) {
        throw std::runtime_error("탐색 뒤에 desired geometry가 바뀌었습니다.");
      }
    }
    std::cout << "P-005 ATTEMPT #3 " << (reproducible ? "DIAGNOSIS COMPLETE" : "INCOMPLETE") << "\n";
    exit_code = reproducible ? 0 : 1;
  } catch (const std::exception& error) {
    stop_spin();
    std::ofstream summary(output_dir + "/capability-summary.json");
    summary << "{\n  \"status\": \"INCOMPLETE\",\n  \"error\": \"" << error.what() << "\"\n}\n";
    std::cout << "P-005 ATTEMPT #3 INCOMPLETE\n" << error.what() << "\n";
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
