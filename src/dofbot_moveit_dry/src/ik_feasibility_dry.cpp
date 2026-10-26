// Copyright 2026 cad-less-robotics contributors
// P-005 attempt 2 진단 전용. production waypoint, motion contract, vendor kinematics.yaml은 바꾸지 않는다.

#include "robot_controller/motion_contract.hpp"
#include "vision_path_generator/polyline.hpp"

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <pluginlib/class_loader.hpp>

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
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <sys/wait.h>
#include <utility>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kAcceptPositionM = 1e-3;
constexpr double kAcceptOrientationDeg = 1.0;
constexpr double kAcceptToolDeg = 1.0;
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

const double kTimeouts[] = {0.0, 0.005, 0.01, 0.05, 0.1, 0.5, 1.0};

struct Query
{
  bool success{false};
  int error_code{0};
  std::string error_name{"NOT_RUN"};
  double solve_time_s{0.0};
  std::vector<double> joints;
  int limit_violations{0};
  bool colliding{false};
  double position_error_m{-1.0};
  double tool_axis_error_deg{-1.0};
  double orientation_error_deg{-1.0};
  std::string result{"NOT_RUN"};
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
};

struct Findings
{
  std::size_t focus_local{1};
  std::size_t focus_source{0};
  bool direct_ik_failed{false};
  std::string sequence_5ms;
  bool full_5ms_init{false};
  bool full_5ms_any{false};
  bool full_1s_init{false};
  bool full_1s_any{false};
  bool full_1s_kinematic{false};
  bool waypoint0_fail_1s{false};
  bool position_kdl{false};
  bool position_kdl_clear{false};
  bool position_dls{false};
  bool position_dls_clear{false};
  bool roll_clear{false};
  bool tool2{false};
  bool tool5{false};
  bool tool10{false};
  bool tool_implied{false};
  int collision_full{0};
  int clear_full{0};
  int limit_full{0};
  int no_ik_full_1s{0};
  double delta_position_m{0.0};
  double delta_tool_deg{0.0};
  double delta_orientation_deg{0.0};
  double delta_roll_deg{0.0};
  std::string successful_rolls;
  std::vector<std::string> labels;
  std::string primary;
  bool pattern{false};
};

class Csv
{
public:
  Csv(const std::string& path, const std::string& header)
  {
    output_.open(path);
    if (!output_) {
      throw std::runtime_error("CSV를 열 수 없습니다: " + path);
    }
    output_ << header << "\n";
  }

  void row(const std::vector<std::string>& cells)
  {
    for (std::size_t index = 0; index < cells.size(); ++index) {
      if (index > 0) {
        output_ << ',';
      }
      output_ << cells[index];
    }
    output_ << "\n";
    output_.flush();
  }

private:
  std::ofstream output_;
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

std::string joinJoints(const std::vector<double>& values)
{
  std::ostringstream output;
  output << std::setprecision(17);
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      output << ';';
    }
    output << values[index];
  }
  return output.str();
}

std::string errorName(int code)
{
  using Codes = moveit_msgs::msg::MoveItErrorCodes;
  if (code == Codes::SUCCESS) {
    return "SUCCESS";
  }
  if (code == Codes::TIMED_OUT) {
    return "TIMED_OUT";
  }
  if (code == Codes::NO_IK_SOLUTION) {
    return "NO_IK_SOLUTION";
  }
  return "CODE_" + std::to_string(code);
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

struct StepDelta
{
  double position_m{0.0};
  double tool_z_deg{0.0};
  double orientation_deg{0.0};
  double roll_deg{0.0};
  double tilt_deg{0.0};
};

StepDelta deltaOf(const robot_controller::MotionPose& from, const robot_controller::MotionPose& to)
{
  StepDelta delta;
  delta.position_m = robot_controller::poseDistance(from, to);
  delta.tool_z_deg = robot_controller::toolAxisAngleDeg(from, to);
  delta.orientation_deg = robot_controller::quaternionStepDeg(from, to);
  Eigen::Quaterniond start(from.qw, from.qx, from.qy, from.qz);
  Eigen::Quaterniond finish(to.qw, to.qx, to.qy, to.qz);
  start.normalize();
  finish.normalize();
  Eigen::Quaterniond local = start.conjugate() * finish;
  if (local.w() < 0.0) {
    local.coeffs() *= -1.0;
  }
  const Eigen::AngleAxisd axis(local);
  if (axis.angle() < 1e-12) {
    return delta;
  }
  delta.roll_deg = axis.axis().z() * axis.angle() * 180.0 / kPi;
  delta.tilt_deg = std::hypot(axis.axis().x(), axis.axis().y()) * axis.angle() * 180.0 / kPi;
  return delta;
}

robot_controller::MotionPose turned(
    const robot_controller::MotionPose& pose, double tilt_rad, double azimuth_rad, double roll_rad)
{
  Eigen::Quaterniond rotation(pose.qw, pose.qx, pose.qy, pose.qz);
  rotation.normalize();
  const Eigen::Vector3d tilt_axis(std::cos(azimuth_rad), std::sin(azimuth_rad), 0.0);
  const Eigen::Quaterniond tilt(Eigen::AngleAxisd(tilt_rad, tilt_axis));
  const Eigen::Quaterniond roll(Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond changed = rotation * tilt * roll;
  changed.normalize();
  robot_controller::MotionPose output = pose;
  output.qx = changed.x();
  output.qy = changed.y();
  output.qz = changed.z();
  output.qw = changed.w();
  return output;
}

geometry_msgs::msg::Pose poseMessage(const robot_controller::MotionPose& pose)
{
  Eigen::Quaterniond rotation(pose.qw, pose.qx, pose.qy, pose.qz);
  rotation.normalize();
  geometry_msgs::msg::Pose message;
  message.position.x = pose.x;
  message.position.y = pose.y;
  message.position.z = pose.z;
  message.orientation.x = rotation.x();
  message.orientation.y = rotation.y();
  message.orientation.z = rotation.z();
  message.orientation.w = rotation.w();
  return message;
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

int limitViolations(const moveit::core::RobotState& state, const std::vector<std::string>& joints)
{
  int violations = 0;
  for (const auto& name : joints) {
    const auto& bounds = state.getRobotModel()->getVariableBounds(name);
    if (!bounds.position_bounded_) {
      continue;
    }
    const double value = state.getVariablePosition(name);
    if (value < bounds.min_position_ - 1e-6 || value > bounds.max_position_ + 1e-6) {
      ++violations;
    }
  }
  return violations;
}

bool stateColliding(const Scene& scene, const moveit::core::RobotState& state)
{
  planning_scene_monitor::LockedPlanningSceneRO locked(scene.monitor);
  moveit::core::RobotState copy(state);
  copy.update(true);
  return locked->isStateColliding(copy, "", false);
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
    if (values->size() != scene.joint_names.size()) {
      throw std::runtime_error("IK seed 길이가 arm_group과 다릅니다.");
    }
    state.setJointGroupPositions(scene.arm, *values);
  }
  state.update(true);
  return state;
}

void finishQuery(const Scene& scene, const robot_controller::MotionPose& target, Query& query)
{
  if (!query.success) {
    query.result = "NO_IK";
    return;
  }
  auto state = makeState(scene, "", &query.joints);
  query.limit_violations = limitViolations(state, scene.joint_names);
  query.colliding = stateColliding(scene, state);
  const auto actual = poseFromState(state, scene.end_link->getName());
  query.position_error_m = robot_controller::poseDistance(actual, target);
  query.tool_axis_error_deg = robot_controller::toolAxisAngleDeg(actual, target);
  query.orientation_error_deg = robot_controller::quaternionStepDeg(actual, target);
  if (query.limit_violations > 0) {
    query.result = "IK_FOUND_LIMIT";
  } else if (query.colliding) {
    query.result = "IK_FOUND_COLLISION";
  } else {
    query.result = "IK_FOUND_CLEAR";
  }
}

Query runKdl(
    const Scene& scene, const kinematics::KinematicsBase& solver, const robot_controller::MotionPose& target,
    const std::vector<double>& seed, double timeout)
{
  Query query;
  auto message = poseMessage(target);
  moveit_msgs::msg::MoveItErrorCodes code;
  const auto begun = std::chrono::steady_clock::now();
  query.success = solver.searchPositionIK(message, seed, timeout, query.joints, code);
  query.solve_time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - begun).count();
  query.error_code = code.val;
  query.error_name = errorName(code.val);
  finishQuery(scene, target, query);
  return query;
}

bool fullAccept(const Query& query)
{
  return query.success && query.limit_violations == 0 && !query.colliding && query.position_error_m < kAcceptPositionM &&
         query.orientation_error_deg < kAcceptOrientationDeg;
}

bool positionAccept(const Query& query)
{
  return query.success && query.limit_violations == 0 && query.position_error_m >= 0.0 &&
         query.position_error_m < kAcceptPositionM;
}

bool toolAccept(const Query& query)
{
  return positionAccept(query) && !query.colliding && query.tool_axis_error_deg < kAcceptToolDeg;
}

int jacobianLinearOffset(const Scene& scene)
{
  auto state = makeState(scene, "init", nullptr);
  Eigen::MatrixXd jacobian;
  if (!state.getJacobian(scene.arm, scene.end_link, Eigen::Vector3d::Zero(), jacobian)) {
    return -1;
  }
  if (jacobian.rows() < 6 || jacobian.cols() < 1) {
    return -1;
  }
  const auto seed = positionsOf(state, scene.joint_names);
  const std::size_t index = seed.size() > 1 ? 1 : 0;
  const double delta = 1e-6;
  auto shifted = seed;
  shifted[index] += delta;
  auto perturbed = makeState(scene, "init", &shifted);
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

Query runPositionDls(const Scene& scene, const robot_controller::MotionPose& target, const std::string& arm_state, int row)
{
  Query query;
  query.error_name = "DLS_NO_CONVERGENCE";
  if (row < 0) {
    query.error_name = "DLS_JACOBIAN_UNCHECKED";
    query.result = "NO_IK";
    return query;
  }
  auto state = makeState(scene, arm_state, nullptr);
  const Eigen::Vector3d goal(target.x, target.y, target.z);
  const auto begun = std::chrono::steady_clock::now();
  for (int iteration = 0; iteration < 200; ++iteration) {
    state.update(true);
    const Eigen::Vector3d error = goal - state.getGlobalLinkTransform(scene.end_link).translation();
    if (error.norm() < 1e-4) {
      query.success = true;
      query.error_name = "DLS_SUCCESS";
      break;
    }
    Eigen::MatrixXd jacobian;
    if (!state.getJacobian(scene.arm, scene.end_link, Eigen::Vector3d::Zero(), jacobian)) {
      break;
    }
    const Eigen::MatrixXd linear = jacobian.block(row, 0, 3, jacobian.cols());
    const Eigen::Matrix3d normal = linear * linear.transpose() + 1e-4 * Eigen::Matrix3d::Identity();
    Eigen::VectorXd step = linear.transpose() * normal.ldlt().solve(error);
    const double norm = step.norm();
    if (norm > 0.25) {
      step *= 0.25 / norm;
    }
    auto values = positionsOf(state, scene.joint_names);
    for (std::size_t index = 0; index < values.size(); ++index) {
      const auto& bounds = scene.model->getVariableBounds(scene.joint_names[index]);
      double next = values[index] + step(static_cast<Eigen::Index>(index));
      if (bounds.position_bounded_) {
        next = std::min(bounds.max_position_, std::max(bounds.min_position_, next));
      }
      values[index] = next;
    }
    state.setJointGroupPositions(scene.arm, values);
    if (step.norm() < 1e-8) {
      break;
    }
  }
  query.solve_time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - begun).count();
  query.joints = positionsOf(state, scene.joint_names);
  if (query.success) {
    finishQuery(scene, target, query);
    query.error_name = "DLS_SUCCESS";
  } else {
    query.joints.clear();
    query.result = "NO_IK";
  }
  return query;
}

void writeLimits(
    Csv& csv, const std::string& case_id, std::size_t source_index, const std::string& query_name,
    const std::string& seed, const Scene& scene, const std::vector<double>& values)
{
  if (values.size() != scene.joint_names.size()) {
    csv.row({case_id, std::to_string(source_index), query_name, seed, "", "", "", "", "", "", "no_solution"});
    return;
  }
  for (std::size_t index = 0; index < values.size(); ++index) {
    const auto& bounds = scene.model->getVariableBounds(scene.joint_names[index]);
    const double lower = bounds.position_bounded_ ? bounds.min_position_ : 0.0;
    const double upper = bounds.position_bounded_ ? bounds.max_position_ : 0.0;
    const bool violates = bounds.position_bounded_ && (values[index] < lower - 1e-6 || values[index] > upper + 1e-6);
    csv.row(
        {case_id, std::to_string(source_index), query_name, seed, scene.joint_names[index], num(values[index]),
         num(lower), num(upper), num(values[index] - lower), num(upper - values[index]), flag(violates)});
  }
}

std::vector<std::string> queryCells(
    const std::string& case_id, std::size_t source_index, const std::string& query_name, const std::string& seed,
    double timeout, const Query& query)
{
  return {
      case_id,
      std::to_string(source_index),
      query_name,
      seed,
      num(timeout),
      flag(query.success),
      flag(query_name == "full_pose" ? fullAccept(query) : (positionAccept(query) && !query.colliding)),
      query.error_name,
      num(query.solve_time_s),
      query.result,
      std::to_string(query.limit_violations),
      flag(query.colliding),
      num(query.position_error_m),
      num(query.tool_axis_error_deg),
      num(query.orientation_error_deg)};
}

void noteFull(Findings& findings, const std::string& seed, double timeout, const Query& query)
{
  const bool kinematic = query.success && query.limit_violations == 0 && query.position_error_m < kAcceptPositionM &&
                         query.orientation_error_deg < kAcceptOrientationDeg;
  const bool clear = kinematic && !query.colliding;
  if (query.success && query.limit_violations > 0) {
    ++findings.limit_full;
  }
  if (kinematic && query.colliding) {
    ++findings.collision_full;
  }
  if (clear) {
    ++findings.clear_full;
  }
  if (std::abs(timeout - 0.005) < 1e-12 && clear) {
    findings.full_5ms_any = true;
    if (seed == "init") {
      findings.full_5ms_init = true;
    }
  }
  if (std::abs(timeout - 1.0) < 1e-12) {
    if (kinematic) {
      findings.full_1s_kinematic = true;
    }
    if (clear) {
      findings.full_1s_any = true;
      if (seed == "init") {
        findings.full_1s_init = true;
      }
    } else if (seed == "init") {
      ++findings.no_ik_full_1s;
    }
  }
}

void classify(Findings& findings)
{
  if (findings.waypoint0_fail_1s) {
    findings.labels.push_back("SOLVER_CONFIGURATION");
  }
  if (!findings.full_5ms_any && findings.full_1s_any) {
    findings.labels.push_back("IK_TIMEOUT");
  }
  const bool position = findings.position_kdl || findings.position_dls;
  const bool full_clear = findings.full_5ms_any || findings.full_1s_any;
  if (!position) {
    findings.labels.push_back("POSITION_UNREACHABLE");
  }
  if (findings.collision_full > 0 && findings.clear_full == 0 && findings.full_1s_kinematic) {
    findings.labels.push_back("COLLISION");
  }
  if (findings.limit_full > 0 && findings.clear_full == 0) {
    findings.labels.push_back("JOINT_LIMIT");
  }
  if (position && !full_clear && !findings.roll_clear) {
    findings.labels.push_back("TOOL_AXIS_INFEASIBLE");
  }
  if (position && !full_clear && findings.roll_clear) {
    findings.labels.push_back("ROLL_INFEASIBLE");
    findings.labels.push_back("FULL_POSE_OVERCONSTRAINED");
    findings.pattern = true;
  }
  if (!findings.full_5ms_init && !findings.full_1s_init && full_clear) {
    findings.labels.push_back("SEED_SENSITIVITY");
  }
  if (findings.labels.empty()) {
    if (findings.full_5ms_any) {
      findings.primary = "DIRECT_FULL_POSE_IK_SUCCESS";
      return;
    }
    findings.labels.push_back("UNKNOWN");
  }
  const char* order[] = {
      "SOLVER_CONFIGURATION", "IK_TIMEOUT",        "POSITION_UNREACHABLE", "COLLISION",
      "JOINT_LIMIT",          "TOOL_AXIS_INFEASIBLE", "ROLL_INFEASIBLE",   "FULL_POSE_OVERCONSTRAINED",
      "SEED_SENSITIVITY",     "UNKNOWN"};
  for (const char* name : order) {
    if (std::find(findings.labels.begin(), findings.labels.end(), name) != findings.labels.end()) {
      findings.primary = name;
      break;
    }
  }
}

std::string joinLabels(const std::vector<std::string>& labels)
{
  std::ostringstream output;
  for (std::size_t index = 0; index < labels.size(); ++index) {
    if (index > 0) {
      output << ' ';
    }
    output << labels[index];
  }
  return output.str();
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("ik_feasibility_dry", options);
  if (node->get_parameter("execute_trajectory").as_bool()) {
    RCLCPP_ERROR(node->get_logger(), "execute_trajectory는 false여야 합니다.");
    rclcpp::shutdown();
    return 2;
  }

  const auto output_dir = node->get_parameter("output_dir").as_string();
  const auto waypoints_csv = node->get_parameter("waypoints_csv").as_string();
  const auto snapshot_pcd = node->get_parameter("snapshot_pcd").as_string();
  const auto planning_group = node->get_parameter("planning_group").as_string();
  const auto end_link_name = node->get_parameter("end_effector_link").as_string();
  const auto start_state_name = node->get_parameter("start_state_name").as_string();
  const auto grip_state_name = node->get_parameter("grip_state_name").as_string();
  const double max_step = node->get_parameter("max_orientation_step_deg").as_double();
  const auto window_count = static_cast<std::size_t>(node->get_parameter("window_count").as_int());
  const double max_gap = node->get_parameter("max_segment_gap").as_double();
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
    if (node->get_parameter("expected_snapshot_sha256").as_string() != kSnapshotSha ||
        node->get_parameter("expected_waypoints_sha256").as_string() != kWaypointSha) {
      throw std::runtime_error("파라미터의 SHA256이 P-004 고정값과 다릅니다.");
    }
    if (sha256File(snapshot_pcd) != kSnapshotSha || sha256File(waypoints_csv) != kWaypointSha) {
      throw std::runtime_error("snapshot 또는 waypoint SHA256이 P-004 기록과 다릅니다.");
    }
    if (planning_group != "arm_group" || end_link_name != "Arm5_Link" || start_state_name != "init" ||
        grip_state_name != "open" || std::abs(max_step - 45.0) > 1e-12 || window_count != 11) {
      throw std::runtime_error("진단이 production 계획 설정을 바꾸려 합니다.");
    }
    const auto poses = loadWaypoints(waypoints_csv);
    const auto pairs = assessPairs(poses, max_gap);
    const auto validation = robot_controller::validateMotionInput(poses, pairs, max_step);
    if (!validation.error.empty()) {
      throw std::runtime_error(validation.error);
    }
    std::string window_error;
    const auto windows =
        robot_controller::selectPlanningWindows(poses, validation, window_count, max_step, window_error);
    if (!window_error.empty() || windows.size() != 3) {
      throw std::runtime_error(window_error.empty() ? "계획 창 개수가 3이 아닙니다." : window_error);
    }
    for (std::size_t index = 0; index < 3; ++index) {
      if (windows[index].case_id != kFrozenCases[index].id ||
          windows[index].start_index != kFrozenCases[index].start ||
          windows[index].end_index != kFrozenCases[index].end) {
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
      const auto type = item.second->type;
      if (type == urdf::Joint::FIXED || type == urdf::Joint::UNKNOWN) {
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
    scene.arm = scene.model->getJointModelGroup(planning_group);
    scene.grip = scene.model->getJointModelGroup("grip_group");
    scene.end_link = scene.model->getLinkModel(end_link_name);
    scene.grip_state = grip_state_name;
    scene.monitor = moveit_cpp->getPlanningSceneMonitor();
    if (scene.arm == nullptr || scene.grip == nullptr || scene.end_link == nullptr || scene.monitor == nullptr) {
      throw std::runtime_error("arm_group, grip_group 또는 Arm5_Link가 없습니다.");
    }
    const auto solver = scene.arm->getSolverInstance();
    if (!solver) {
      throw std::runtime_error("arm_group kinematics solver가 없습니다.");
    }
    scene.joint_names = solver->getJointNames();
    const std::vector<std::string> expected = {
        "Arm1_Joint", "Arm2_Joint", "Arm3_Joint", "Arm4_Joint", "Arm5_Joint"};
    if (scene.arm->getVariableNames() != expected || scene.joint_names != expected) {
      throw std::runtime_error("runtime joint 이름이 Arm1_Joint–Arm5_Joint가 아닙니다.");
    }
    const int linear_row = jacobianLinearOffset(scene);
    const auto init_state = makeState(scene, "init", nullptr);
    const auto init_seed = positionsOf(init_state, scene.joint_names);
    const bool init_collision = stateColliding(scene, init_state);

    {
      std::ofstream model_file(output_dir + "/robot-model.txt");
      model_file << std::setprecision(17);
      model_file << "planning_group " << scene.arm->getName() << "\n";
      model_file << "active_joint_count " << scene.arm->getActiveJointModels().size() << "\n";
      model_file << "variable_count " << scene.arm->getVariableCount() << "\n";
      model_file << "active_joint_names";
      for (const auto& name : scene.arm->getActiveJointModelNames()) {
        model_file << " " << name;
      }
      model_file << "\nvariable_names";
      for (const auto& name : scene.arm->getVariableNames()) {
        model_file << " " << name;
      }
      model_file << "\nend_link " << scene.end_link->getName() << "\n";
      model_file << "solver_group " << solver->getGroupName() << "\n";
      model_file << "solver_base " << solver->getBaseFrame() << "\n";
      model_file << "solver_tip " << solver->getTipFrame() << "\n";
      model_file << "solver_joint_names";
      for (const auto& name : scene.joint_names) {
        model_file << " " << name;
      }
      model_file << "\njacobian_linear_row " << linear_row << "\n";
      model_file << "init_collision " << (init_collision ? "yes" : "no") << "\n";
      for (const char* state_name : {"init", "up", "down"}) {
        const auto state = makeState(scene, state_name, nullptr);
        model_file << "named_state " << state_name;
        const auto values = positionsOf(state, scene.joint_names);
        for (std::size_t index = 0; index < values.size(); ++index) {
          model_file << " " << scene.joint_names[index] << "=" << values[index];
        }
        model_file << " collision " << (stateColliding(scene, state) ? "yes" : "no") << "\n";
      }
    }

    const double parameter_timeout =
        node->get_parameter("robot_description_kinematics.arm_group.kinematics_solver_timeout").as_double();
    const double parameter_resolution =
        node->get_parameter("robot_description_kinematics.arm_group.kinematics_solver_search_resolution").as_double();
    const auto parameter_solver =
        node->get_parameter("robot_description_kinematics.arm_group.kinematics_solver").as_string();
    {
      std::ofstream config(output_dir + "/kinematics-config.txt");
      config << std::setprecision(17);
      config << "vendor_file " << node->get_parameter("kinematics_yaml_path").as_string() << "\n";
      config << "vendor_text_begin\n";
      config << readText(node->get_parameter("kinematics_yaml_path").as_string());
      config << "vendor_text_end\n";
      config << "runtime_solver " << parameter_solver << "\n";
      config << "runtime_parameter_timeout_s " << parameter_timeout << "\n";
      config << "runtime_parameter_search_resolution " << parameter_resolution << "\n";
      config << "runtime_group_default_ik_timeout_s " << scene.arm->getDefaultIKTimeout() << "\n";
      config << "runtime_position_only_ik_on_production_solver no\n";
      config << "runtime_max_solver_iterations_parameter "
             << (node->has_parameter("robot_description_kinematics.arm_group.max_solver_iterations") ? "set" : "absent_code_default_500")
             << "\n";
      config << "runtime_epsilon_parameter "
             << (node->has_parameter("robot_description_kinematics.arm_group.epsilon") ? "set" : "absent_code_default_1e-5")
             << "\n";
      config << "kdl_failure_code_note Humble KDL searchPositionIK sets TIMED_OUT when the search loop ends without a solution, including when CartToJnt cannot reach the target. The timeout sweep, not the error name, separates IK_TIMEOUT from an exhausted search.\n";
      config << "source https://raw.githubusercontent.com/moveit/moveit2/humble/moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp\n";
      config << "production_yaml_modified no\n";
    }

    pluginlib::ClassLoader<kinematics::KinematicsBase> loader("moveit_core", "kinematics::KinematicsBase");
    rclcpp::Node::SharedPtr position_node;
    kinematics::KinematicsBasePtr position_solver;
    try {
      position_node = std::make_shared<rclcpp::Node>("kdl_position_only_diagnostic");
      position_node->declare_parameter("arm_group.position_only_ik", true);
      position_node->declare_parameter("arm_group.max_solver_iterations", 500);
      position_node->declare_parameter("arm_group.epsilon", 1e-5);
      position_node->declare_parameter("arm_group.orientation_vs_position", 1.0);
      position_solver = loader.createSharedInstance("kdl_kinematics_plugin/KDLKinematicsPlugin");
      const std::vector<std::string> tips = {solver->getTipFrame()};
      if (!position_solver->initialize(
              position_node, *scene.model, planning_group, solver->getBaseFrame(), tips, parameter_resolution)) {
        position_solver.reset();
      }
    } catch (const std::exception& error) {
      position_solver.reset();
      RCLCPP_ERROR(node->get_logger(), "position-only KDL 진단을 만들지 못했습니다: %s", error.what());
    }
    {
      std::ofstream config(output_dir + "/kinematics-config.txt", std::ios::app);
      config << "diagnostic_position_only_kdl " << (position_solver ? "yes" : "no") << "\n";
      config << "diagnostic_position_only_written_to_vendor_yaml no\n";
    }

    const auto anchor = poseFromState(init_state, end_link_name);
    std::ofstream placement_check(output_dir + "/placement-check.txt");
    placement_check << std::setprecision(17);
    double worst_placement = 0.0;
    std::vector<std::vector<robot_controller::MotionPose>> placed_cases;
    for (std::size_t index = 0; index < 3; ++index) {
      const auto& frozen = kFrozenCases[index];
      const auto computed = robot_controller::rigidPlacement(poses[frozen.start], anchor);
      double worst = 0.0;
      for (int axis = 0; axis < 3; ++axis) {
        worst = std::max(worst, std::abs(computed.translation[axis] - frozen.translation[axis]));
      }
      for (int cell = 0; cell < 9; ++cell) {
        worst = std::max(worst, std::abs(computed.rotation[cell] - frozen.rotation[cell]));
      }
      worst_placement = std::max(worst_placement, worst);
      robot_controller::RigidPlacement used = computed;
      for (int axis = 0; axis < 3; ++axis) {
        used.translation[axis] = frozen.translation[axis];
      }
      for (int cell = 0; cell < 9; ++cell) {
        used.rotation[cell] = frozen.rotation[cell];
      }
      used.scale = 1.0;
      std::vector<robot_controller::MotionPose> placed;
      for (std::size_t source = frozen.start; source <= frozen.end; ++source) {
        placed.push_back(robot_controller::applyRigidPlacement(used, poses[source]));
      }
      const double residual = robot_controller::poseDistance(placed.front(), anchor);
      worst_placement = std::max(worst_placement, residual);
      placement_check << frozen.id << " max_abs_diff " << worst << " residual_m " << residual
                      << " scale " << used.scale << "\n";
      placed_cases.push_back(std::move(placed));
    }
    placement_check << "threshold_m 1e-9\nmatch " << (worst_placement < 1e-9 ? "yes" : "no") << "\n";
    if (worst_placement >= 1e-9) {
      throw std::runtime_error("attempt 1 synthetic placement와 재계산값이 다릅니다.");
    }

    const std::string waypoint_header =
        "local_index,source_index,x,y,z,qx,qy,qz,qw,seed,timeout_s,solver_success,accepted,error_name,solve_time_s,result,limit_violations,colliding,fk_position_error_m,fk_tool_axis_error_deg,fk_orientation_error_deg,joints";
    std::vector<std::unique_ptr<Csv>> waypoint_csv;
    waypoint_csv.push_back(std::make_unique<Csv>(output_dir + "/case-01-waypoint-ik.csv", waypoint_header));
    waypoint_csv.push_back(std::make_unique<Csv>(output_dir + "/case-02-waypoint-ik.csv", waypoint_header));
    waypoint_csv.push_back(std::make_unique<Csv>(output_dir + "/case-03-waypoint-ik.csv", waypoint_header));
    Csv timeout_csv(
        output_dir + "/timeout-sweep.csv",
        "case_id,source_index,query,seed,timeout_s,solver_success,accepted,error_name,solve_time_s,result,limit_violations,colliding,fk_position_error_m,fk_tool_axis_error_deg,fk_orientation_error_deg");
    Csv seed_csv(
        output_dir + "/seed-sweep.csv",
        "case_id,source_index,query,seed,timeout_s,solver_success,accepted,error_name,solve_time_s,result,limit_violations,colliding,fk_position_error_m,fk_tool_axis_error_deg,fk_orientation_error_deg");
    Csv orientation_csv(
        output_dir + "/orientation-analysis.csv",
        "case_id,from_index,to_index,source_delta_position_m,placed_delta_position_m,source_tool_z_deg,placed_tool_z_deg,source_orientation_deg,placed_orientation_deg,source_roll_deg,placed_roll_deg,source_tilt_deg,placed_tilt_deg");
    Csv roll_csv(
        output_dir + "/roll-relaxation.csv",
        "case_id,source_index,seed,roll_deg,step_deg,timeout_s,solver_success,accepted,error_name,solve_time_s,result,limit_violations,colliding,fk_position_error_m,fk_tool_axis_error_deg,fk_orientation_error_deg,joints");
    Csv tool_csv(
        output_dir + "/tool-axis-relaxation.csv",
        "case_id,source_index,seed,alpha_deg,azimuth_deg,roll_deg,timeout_s,solver_success,accepted,achieved_tool_axis_delta_deg,error_name,solve_time_s,result,fk_position_error_m,fk_tool_axis_error_deg,note");
    Csv matrix_csv(
        output_dir + "/ik-feasibility-matrix.csv",
        "case_id,source_index,query,timeout_s,seed,result,solve_time_s,limit_violations,colliding,fk_position_error_m,fk_tool_axis_error_deg,fk_orientation_error_deg,detail");
    Csv fk_csv(
        output_dir + "/fk-validation.csv",
        "case_id,source_index,query,seed,timeout_s,solver_success,result,fk_position_error_m,fk_tool_axis_error_deg,fk_orientation_error_deg");
    Csv limit_csv(
        output_dir + "/joint-limit-analysis.csv",
        "case_id,source_index,query,seed,joint,value,lower,upper,margin_to_lower,margin_to_upper,violates");
    Csv collision_csv(
        output_dir + "/collision-analysis.csv",
        "case_id,source_index,query,seed,timeout_s,ik_result,colliding");

    auto recordFk = [&](const std::string& case_id, std::size_t source, const std::string& query_name,
                        const std::string& seed, double timeout, const Query& query) {
      if (!query.success) {
        return;
      }
      fk_csv.row(
          {case_id, std::to_string(source), query_name, seed, num(timeout), flag(query.success), query.result,
           num(query.position_error_m), num(query.tool_axis_error_deg), num(query.orientation_error_deg)});
    };

    std::vector<Findings> reports;
    for (std::size_t case_index = 0; case_index < 3; ++case_index) {
      const auto& frozen = kFrozenCases[case_index];
      const auto& placed = placed_cases[case_index];
      Findings findings;
      std::optional<std::vector<double>> previous;
      std::vector<std::optional<std::vector<double>>> accepted(placed.size());
      for (std::size_t local = 0; local + 1 < placed.size(); ++local) {
        const auto source_delta = deltaOf(poses[frozen.start + local], poses[frozen.start + local + 1]);
        const auto placed_delta = deltaOf(placed[local], placed[local + 1]);
        orientation_csv.row(
            {frozen.id, std::to_string(frozen.start + local), std::to_string(frozen.start + local + 1),
             num(source_delta.position_m), num(placed_delta.position_m), num(source_delta.tool_z_deg),
             num(placed_delta.tool_z_deg), num(source_delta.orientation_deg), num(placed_delta.orientation_deg),
             num(source_delta.roll_deg), num(placed_delta.roll_deg), num(source_delta.tilt_deg),
             num(placed_delta.tilt_deg)});
      }

      auto writeWaypoint = [&](std::size_t local, const std::string& seed, double timeout, const Query& query) {
        const auto& pose = placed[local];
        waypoint_csv[case_index]->row(
            {std::to_string(local), std::to_string(pose.source_index), num(pose.x), num(pose.y), num(pose.z),
             num(pose.qx), num(pose.qy), num(pose.qz), num(pose.qw), seed, num(timeout), flag(query.success),
             flag(fullAccept(query)), query.error_name, num(query.solve_time_s), query.result,
             std::to_string(query.limit_violations), flag(query.colliding), num(query.position_error_m),
             num(query.tool_axis_error_deg), num(query.orientation_error_deg), joinJoints(query.joints)});
      };

      for (std::size_t local = 0; local < placed.size(); ++local) {
        const Query init_short = runKdl(scene, *solver, placed[local], init_seed, 0.005);
        writeWaypoint(local, "init", 0.005, init_short);
        timeout_csv.row(queryCells(frozen.id, placed[local].source_index, "full_pose", "init", 0.005, init_short));
        recordFk(frozen.id, placed[local].source_index, "full_pose", "init", 0.005, init_short);
        findings.sequence_5ms.push_back(fullAccept(init_short) ? 'S' : 'F');
        if (fullAccept(init_short)) {
          accepted[local] = init_short.joints;
        }
        if (previous.has_value()) {
          const Query chained = runKdl(scene, *solver, placed[local], *previous, 0.005);
          writeWaypoint(local, "previous", 0.005, chained);
          timeout_csv.row(queryCells(frozen.id, placed[local].source_index, "full_pose", "previous", 0.005, chained));
          recordFk(frozen.id, placed[local].source_index, "full_pose", "previous", 0.005, chained);
          if (fullAccept(chained)) {
            accepted[local] = chained.joints;
          }
        }
        previous = accepted[local];
        RCLCPP_INFO(
            node->get_logger(), "%s local %zu source %zu full_pose/0.005 %s", frozen.id, local,
            placed[local].source_index, init_short.result.c_str());
      }

      findings.direct_ik_failed = findings.sequence_5ms.find('F') != std::string::npos;
      findings.focus_local = findings.direct_ik_failed ? findings.sequence_5ms.find('F') : 1;
      findings.focus_source = placed[findings.focus_local].source_index;
      const auto focus_previous = findings.focus_local == 0 ? std::nullopt : accepted[findings.focus_local - 1];
      if (findings.focus_local > 0) {
        const auto step = deltaOf(poses[findings.focus_source - 1], poses[findings.focus_source]);
        findings.delta_position_m = step.position_m;
        findings.delta_tool_deg = step.tool_z_deg;
        findings.delta_orientation_deg = step.orientation_deg;
        findings.delta_roll_deg = step.roll_deg;
      }
      const robot_controller::MotionPose& focus_pose = placed[findings.focus_local];

      auto sweepSeed = [&](const std::string& seed_name, const std::vector<double>& seed, bool full_grid) {
        for (double timeout : kTimeouts) {
          if (!full_grid && std::abs(timeout - 0.005) > 1e-12 && std::abs(timeout - 0.1) > 1e-12 &&
              std::abs(timeout - 1.0) > 1e-12 && timeout != 0.0) {
            continue;
          }
          const Query query = runKdl(scene, *solver, focus_pose, seed, timeout);
          timeout_csv.row(queryCells(frozen.id, findings.focus_source, "full_pose", seed_name, timeout, query));
          seed_csv.row(queryCells(frozen.id, findings.focus_source, "full_pose", seed_name, timeout, query));
          noteFull(findings, seed_name, timeout, query);
          recordFk(frozen.id, findings.focus_source, "full_pose", seed_name, timeout, query);
          collision_csv.row(
              {frozen.id, std::to_string(findings.focus_source), "full_pose", seed_name, num(timeout), query.result,
               flag(query.colliding)});
          if (query.success) {
            writeLimits(limit_csv, frozen.id, findings.focus_source, "full_pose", seed_name, scene, query.joints);
          }
        }
        writeLimits(limit_csv, frozen.id, findings.focus_source, "seed_state", seed_name, scene, seed);
      };

      sweepSeed("init", init_seed, true);
      if (focus_previous.has_value()) {
        sweepSeed("previous", *focus_previous, true);
      }
      sweepSeed("up", positionsOf(makeState(scene, "up", nullptr), scene.joint_names), false);
      sweepSeed("down", positionsOf(makeState(scene, "down", nullptr), scene.joint_names), false);
      if (findings.focus_local == 0 && !findings.full_1s_init) {
        findings.waypoint0_fail_1s = true;
      }

      auto sweepPosition = [&](const std::string& seed_name, const std::vector<double>& seed, double timeout) {
        if (!position_solver) {
          return;
        }
        const Query query = runKdl(scene, *position_solver, focus_pose, seed, timeout);
        timeout_csv.row(queryCells(frozen.id, findings.focus_source, "position_only_kdl", seed_name, timeout, query));
        const bool clear = positionAccept(query) && !query.colliding;
        if (positionAccept(query)) {
          findings.position_kdl = true;
        }
        if (clear) {
          findings.position_kdl_clear = true;
        }
        recordFk(frozen.id, findings.focus_source, "position_only_kdl", seed_name, timeout, query);
        collision_csv.row(
            {frozen.id, std::to_string(findings.focus_source), "position_only_kdl", seed_name, num(timeout),
             query.result, flag(query.colliding)});
        if (query.success) {
          writeLimits(limit_csv, frozen.id, findings.focus_source, "position_only_kdl", seed_name, scene, query.joints);
        }
      };
      for (double timeout : {0.005, 0.1, 1.0}) {
        sweepPosition("init", init_seed, timeout);
        if (focus_previous.has_value()) {
          sweepPosition("previous", *focus_previous, timeout);
        }
      }
      sweepPosition("up", positionsOf(makeState(scene, "up", nullptr), scene.joint_names), 1.0);
      sweepPosition("down", positionsOf(makeState(scene, "down", nullptr), scene.joint_names), 1.0);
      if (position_solver) {
        const Query positioned = runKdl(scene, *position_solver, focus_pose, init_seed, 0.1);
        if (positioned.success && positioned.joints.size() == scene.joint_names.size()) {
          for (double timeout : {0.005, 0.1, 1.0}) {
            const Query query = runKdl(scene, *solver, focus_pose, positioned.joints, timeout);
            timeout_csv.row(
                queryCells(frozen.id, findings.focus_source, "full_pose", "position_only_init", timeout, query));
            seed_csv.row(queryCells(frozen.id, findings.focus_source, "full_pose", "position_only_init", timeout, query));
            noteFull(findings, "position_only_init", timeout, query);
            recordFk(frozen.id, findings.focus_source, "full_pose", "position_only_init", timeout, query);
            collision_csv.row(
                {frozen.id, std::to_string(findings.focus_source), "full_pose", "position_only_init", num(timeout),
                 query.result, flag(query.colliding)});
            if (query.success) {
              writeLimits(
                  limit_csv, frozen.id, findings.focus_source, "full_pose", "position_only_init", scene, query.joints);
            }
          }
        }
      }

      for (const char* seed_name : {"init", "up", "down"}) {
        const Query dls = runPositionDls(scene, focus_pose, seed_name, linear_row);
        timeout_csv.row(queryCells(frozen.id, findings.focus_source, "position_only_dls", seed_name, 0.0, dls));
        if (positionAccept(dls)) {
          findings.position_dls = true;
          if (!dls.colliding) {
            findings.position_dls_clear = true;
          }
        }
        recordFk(frozen.id, findings.focus_source, "position_only_dls", seed_name, 0.0, dls);
        collision_csv.row(
            {frozen.id, std::to_string(findings.focus_source), "position_only_dls", seed_name, "0", dls.result,
             flag(dls.colliding)});
      }

      for (const char* seed_name : {"init", "previous"}) {
        if (std::string(seed_name) == "previous" && !focus_previous.has_value()) {
          continue;
        }
        const auto& seed = std::string(seed_name) == "init" ? init_seed : *focus_previous;
        for (int step = -12; step <= 11; ++step) {
          const double roll_deg = static_cast<double>(step) * 15.0;
          const auto varied = turned(focus_pose, 0.0, 0.0, roll_deg * kPi / 180.0);
          const Query query = runKdl(scene, *solver, varied, seed, 0.1);
          const bool accepted = toolAccept(query);
          if (accepted) {
            findings.roll_clear = true;
            if (!findings.successful_rolls.empty()) {
              findings.successful_rolls.push_back(' ');
            }
            findings.successful_rolls += seed_name;
            findings.successful_rolls.push_back(':');
            findings.successful_rolls += num(roll_deg);
          }
          roll_csv.row(
              {frozen.id, std::to_string(findings.focus_source), seed_name, num(roll_deg), "15", "0.1",
               flag(query.success), flag(accepted), query.error_name, num(query.solve_time_s), query.result,
               std::to_string(query.limit_violations), flag(query.colliding), num(query.position_error_m),
               num(query.tool_axis_error_deg), num(query.orientation_error_deg), joinJoints(query.joints)});
          recordFk(frozen.id, findings.focus_source, "roll_relaxed", seed_name, 0.1, query);
        }
      }
      if (!findings.roll_clear) {
        for (int step = -36; step <= 35; ++step) {
          const double roll_deg = static_cast<double>(step) * 5.0;
          const auto varied = turned(focus_pose, 0.0, 0.0, roll_deg * kPi / 180.0);
          const Query query = runKdl(scene, *solver, varied, init_seed, 0.05);
          const bool accepted = toolAccept(query);
          if (accepted) {
            findings.roll_clear = true;
            findings.successful_rolls = std::string("init5:") + num(roll_deg);
            roll_csv.row(
                {frozen.id, std::to_string(findings.focus_source), "init", num(roll_deg), "5", "0.05",
                 flag(query.success), flag(accepted), query.error_name, num(query.solve_time_s), query.result,
                 std::to_string(query.limit_violations), flag(query.colliding), num(query.position_error_m),
                 num(query.tool_axis_error_deg), num(query.orientation_error_deg), joinJoints(query.joints)});
            break;
          }
          roll_csv.row(
              {frozen.id, std::to_string(findings.focus_source), "init", num(roll_deg), "5", "0.05",
               flag(query.success), flag(accepted), query.error_name, num(query.solve_time_s), query.result,
               std::to_string(query.limit_violations), flag(query.colliding), num(query.position_error_m),
               num(query.tool_axis_error_deg), num(query.orientation_error_deg), joinJoints(query.joints)});
        }
      }

      if (findings.roll_clear) {
        findings.tool_implied = true;
        findings.tool2 = true;
        findings.tool5 = true;
        findings.tool10 = true;
        for (double alpha : {2.0, 5.0, 10.0}) {
          tool_csv.row(
              {frozen.id, std::to_string(findings.focus_source), "init", num(alpha), "", "", "", "true", "true",
               "0", "IMPLIED", "0", "IK_FOUND_CLEAR", "", "", "implied_by_exact_tool_axis_success"});
        }
      } else {
        for (double alpha : {2.0, 5.0, 10.0}) {
          bool found = false;
          for (int azimuth_step = 0; azimuth_step < 4 && !found; ++azimuth_step) {
            const double azimuth = static_cast<double>(azimuth_step) * 90.0;
            for (int roll_step = -6; roll_step <= 5 && !found; ++roll_step) {
              const double roll_deg = static_cast<double>(roll_step) * 30.0;
              const auto varied = turned(focus_pose, alpha * kPi / 180.0, azimuth * kPi / 180.0, roll_deg * kPi / 180.0);
              const double achieved = robot_controller::toolAxisAngleDeg(focus_pose, varied);
              const Query query = runKdl(scene, *solver, varied, init_seed, 0.1);
              const bool accepted = toolAccept(query);
              found = accepted;
              tool_csv.row(
                  {frozen.id, std::to_string(findings.focus_source), "init", num(alpha), num(azimuth), num(roll_deg),
                   "0.1", flag(query.success), flag(accepted), num(achieved), query.error_name, num(query.solve_time_s),
                   query.result, num(query.position_error_m), num(query.tool_axis_error_deg),
                   found ? "stopped_after_first_success" : "coarse_90deg_azimuth"});
            }
          }
          if (alpha == 2.0) {
            findings.tool2 = found;
          } else if (alpha == 5.0) {
            findings.tool5 = found;
          } else {
            findings.tool10 = found;
          }
        }
      }

      classify(findings);
      matrix_csv.row(
          {frozen.id, std::to_string(findings.focus_source), "full_pose", "0.005", "init",
           findings.full_5ms_init ? "SUCCESS" : "FAIL", "", "", "", "", "", "", findings.sequence_5ms});
      matrix_csv.row(
          {frozen.id, std::to_string(findings.focus_source), "full_pose", "1.0", "any_tested_seed",
           findings.full_1s_any ? "SUCCESS" : "FAIL", "", "", "", "", "", "", joinLabels(findings.labels)});
      matrix_csv.row(
          {frozen.id, std::to_string(findings.focus_source), "position_only", "1.0", "kdl_or_dls",
           (findings.position_kdl || findings.position_dls) ? "SUCCESS" : "FAIL", "", "", "", "", "", "",
           std::string("kdl=") + (findings.position_kdl ? "yes" : "no") + " dls=" + (findings.position_dls ? "yes" : "no")});
      matrix_csv.row(
          {frozen.id, std::to_string(findings.focus_source), "tool_axis_fixed_roll_free", "0.1", "init_or_previous",
           findings.roll_clear ? "SUCCESS" : "FAIL", "", "", "", "", "", "", findings.successful_rolls});
      matrix_csv.row(
          {frozen.id, std::to_string(findings.focus_source), "tool_axis_tol_5deg_roll_free", findings.tool_implied ? "implied" : "0.1",
           "init", findings.tool5 ? "SUCCESS" : "FAIL", "", "", "", "", "", "",
           findings.tool_implied ? "implied_by_exact_tool_axis_success" : "coarse_sample"});
      reports.push_back(findings);
      RCLCPP_INFO(
          node->get_logger(), "%s focus %zu primary %s", frozen.id, findings.focus_source, findings.primary.c_str());
    }

    bool same = true;
    for (std::size_t index = 1; index < reports.size(); ++index) {
      if (reports[index].labels != reports.front().labels) {
        same = false;
      }
    }
    {
      std::ofstream json(output_dir + "/root-cause.json");
      json << std::setprecision(17);
      json << "{\n  \"status\": \"DIAGNOSIS_COMPLETE\",\n  \"same_label_set\": " << (same ? "true" : "false") << ",\n";
      json << "  \"cases\": [\n";
      for (std::size_t index = 0; index < reports.size(); ++index) {
        const auto& item = reports[index];
        json << "    {\"case_id\": \"" << kFrozenCases[index].id << "\", \"focus_source_index\": " << item.focus_source
             << ", \"direct_ik_failed\": " << (item.direct_ik_failed ? "true" : "false")
             << ", \"sequence_init_0_005\": \"" << item.sequence_5ms << "\", \"delta_position_m\": " << item.delta_position_m
             << ", \"delta_tool_z_deg\": " << item.delta_tool_deg
             << ", \"delta_orientation_deg\": " << item.delta_orientation_deg
             << ", \"delta_roll_deg\": " << item.delta_roll_deg
             << ", \"full_pose_0_005_clear\": " << (item.full_5ms_any ? "true" : "false")
             << ", \"full_pose_1_0_clear\": " << (item.full_1s_any ? "true" : "false")
             << ", \"position_kdl\": " << (item.position_kdl ? "true" : "false")
             << ", \"position_dls\": " << (item.position_dls ? "true" : "false")
             << ", \"roll_clear\": " << (item.roll_clear ? "true" : "false")
             << ", \"tool_axis_5deg\": " << (item.tool5 ? "true" : "false")
             << ", \"position_and_tool_axis_reachable_full_quaternion_unreachable\": "
             << (item.pattern ? "true" : "false") << ", \"labels\": [";
        for (std::size_t label = 0; label < item.labels.size(); ++label) {
          if (label > 0) {
            json << ", ";
          }
          json << "\"" << item.labels[label] << "\"";
        }
        json << "], \"primary\": \"" << item.primary << "\"}" << (index + 1 == reports.size() ? "\n" : ",\n");
      }
      json << "  ],\n  \"production_geometry_changed\": false,\n  \"production_motion_contract_changed\": false,\n"
           << "  \"vendor_kinematics_yaml_modified\": false,\n  \"camera_robot_calibration\": 0,\n"
           << "  \"fake_tf\": 0,\n  \"execute_calls\": 0,\n  \"controller_sends\": 0\n}\n";
    }
    stop_spin();
    std::cout << "P-005 ATTEMPT #2 DIAGNOSIS COMPLETE\n";
    for (std::size_t index = 0; index < reports.size(); ++index) {
      std::cout << kFrozenCases[index].id << " " << reports[index].primary << " " << reports[index].sequence_5ms
                << "\n";
    }
    exit_code = 0;
  } catch (const std::exception& error) {
    stop_spin();
    std::ofstream json(output_dir + "/root-cause.json");
    json << "{\n  \"status\": \"INCOMPLETE\",\n  \"error\": \"" << error.what() << "\"\n}\n";
    std::cout << "P-005 ATTEMPT #2 INCOMPLETE\n" << error.what() << "\n";
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
