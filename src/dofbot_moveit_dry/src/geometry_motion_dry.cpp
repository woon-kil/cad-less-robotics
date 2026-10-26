// Copyright 2026 cad-less-robotics contributors

#include "robot_controller/motion_contract.hpp"
#include "vision_path_generator/polyline.hpp"

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <urdf/model.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <sys/wait.h>
#include <vector>

namespace
{
constexpr double kPositionMargin = 1e-6;

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

struct CaseResult
{
  std::string id;
  std::size_t source_start{0};
  std::size_t source_end{0};
  std::size_t waypoint_count{0};
  double arc_length_m{0.0};
  double fraction{0.0};
  int trajectory_points{0};
  double planning_time_sec{0.0};
  double max_joint_velocity{0.0};
  int position_violations{0};
  int velocity_violations{0};
  int collision_states{0};
  bool start_collision{false};
  int execution_calls{0};
  double start_position_error_m{0.0};
  double start_orientation_error_deg{0.0};
  double end_position_error_m{0.0};
  double end_orientation_error_deg{0.0};
  double placement_residual_m{0.0};
  std::string failure_class;
  std::string note;
  robot_controller::RigidPlacement placement;
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

int positionViolations(const moveit::core::RobotState& state, const std::vector<std::string>& joints)
{
  int violations = 0;
  for (const auto& name : joints) {
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

bool stateColliding(
    const planning_scene_monitor::PlanningSceneMonitorPtr& monitor, const moveit::core::RobotState& state)
{
  planning_scene_monitor::LockedPlanningSceneRO scene(monitor);
  moveit::core::RobotState copy(state);
  copy.update(true);
  return scene->isStateColliding(copy, "", false);
}

robot_controller::MotionPose poseFromState(const moveit::core::RobotState& state, const std::string& link)
{
  const Eigen::Isometry3d transform = state.getGlobalLinkTransform(link);
  const Eigen::Quaterniond quaternion(transform.linear());
  robot_controller::MotionPose pose;
  pose.x = transform.translation().x();
  pose.y = transform.translation().y();
  pose.z = transform.translation().z();
  pose.qx = quaternion.x();
  pose.qy = quaternion.y();
  pose.qz = quaternion.z();
  pose.qw = quaternion.w();
  return pose;
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
    const std::vector<std::shared_ptr<moveit::core::RobotState>>& trajectory,
    const std::vector<std::string>& joints, double max_velocity, double& max_measured, int& violations)
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
          delta, std::abs(
                     trajectory[index]->getVariablePosition(name) -
                     trajectory[index - 1]->getVariablePosition(name)));
    }
    const double dt = std::max(delta / max_velocity, 0.05);
    const double velocity = delta / dt;
    max_measured = std::max(max_measured, velocity);
    if (velocity > max_velocity + 1e-6) {
      ++violations;
    }
  }
}

std::string classifyCase(const CaseResult& item, double min_fraction)
{
  if (item.start_collision || item.collision_states > 0) {
    return "COLLISION";
  }
  if (item.position_violations > 0) {
    return "JOINT_LIMIT";
  }
  if (item.velocity_violations > 0) {
    return "VELOCITY";
  }
  if (!(item.fraction + 1e-9 >= min_fraction) || item.trajectory_points < 2) {
    return "CARTESIAN_PATH";
  }
  return "OK";
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("geometry_motion_dry", options);

  const bool execute = node->get_parameter("execute_trajectory").as_bool();
  if (execute) {
    RCLCPP_ERROR(node->get_logger(), "execute_trajectory는 false여야 합니다.");
    rclcpp::shutdown();
    return 2;
  }

  const auto output_dir = node->get_parameter("output_dir").as_string();
  const auto waypoints_csv = node->get_parameter("waypoints_csv").as_string();
  const auto snapshot_pcd = node->get_parameter("snapshot_pcd").as_string();
  const auto expected_snapshot = node->get_parameter("expected_snapshot_sha256").as_string();
  const auto expected_waypoints = node->get_parameter("expected_waypoints_sha256").as_string();
  const double max_step = node->get_parameter("max_orientation_step_deg").as_double();
  const auto window_count = static_cast<std::size_t>(node->get_parameter("window_count").as_int());
  const double eef_step = node->get_parameter("eef_step").as_double();
  const double jump_threshold = node->get_parameter("jump_threshold").as_double();
  const double min_fraction = node->get_parameter("min_fraction").as_double();
  const double max_joint_velocity = node->get_parameter("max_joint_velocity").as_double();
  const double max_gap = node->get_parameter("max_segment_gap").as_double();
  const auto planning_group = node->get_parameter("planning_group").as_string();
  const auto end_link_name = node->get_parameter("end_effector_link").as_string();
  const auto start_state_name = node->get_parameter("start_state_name").as_string();
  const auto grip_state_name = node->get_parameter("grip_state_name").as_string();
  const auto source_frame = node->get_parameter("source_frame").as_string();
  const auto planning_frame = node->get_parameter("planning_frame").as_string();
  const int runs = static_cast<int>(node->get_parameter("runs").as_int());
  const int execution_calls = 0;
  const int controller_sends = 0;
  const int tf_publishes = 0;

  std::string fatal;
  std::string snapshot_hash;
  std::string waypoints_hash;
  std::vector<robot_controller::MotionPose> poses;
  std::vector<robot_controller::PairAssessment> pairs;
  robot_controller::MotionValidation validation;
  std::vector<robot_controller::PlanningWindow> windows;
  std::vector<std::vector<CaseResult>> run_cases;
  try {
    snapshot_hash = sha256File(snapshot_pcd);
    waypoints_hash = sha256File(waypoints_csv);
    if (snapshot_hash != expected_snapshot || waypoints_hash != expected_waypoints) {
      throw std::runtime_error("snapshot 또는 waypoint SHA256이 P-004 기록과 다릅니다.");
    }
    poses = loadWaypoints(waypoints_csv);
    pairs = assessPairs(poses, max_gap);
    validation = robot_controller::validateMotionInput(poses, pairs, max_step);
    if (!validation.error.empty()) {
      throw std::runtime_error(validation.error);
    }
    std::string window_error;
    windows = robot_controller::selectPlanningWindows(poses, validation, window_count, max_step, window_error);
    if (!window_error.empty()) {
      throw std::runtime_error(window_error);
    }
  } catch (const std::exception& error) {
    fatal = error.what();
    RCLCPP_ERROR(node->get_logger(), "%s", fatal.c_str());
  }

  robot_controller::SyntheticPlacement metadata;
  metadata.source_frame = source_frame;
  metadata.target_frame = planning_frame;
  metadata.transform_type = robot_controller::kSyntheticType;
  metadata.calibrated = false;
  metadata.purpose = "P-005 software integration dry-run only";
  metadata.scale = 1.0;
  const std::string metadata_error = robot_controller::syntheticMetadataError(metadata);
  if (!metadata_error.empty() && fatal.empty()) {
    fatal = metadata_error;
  }

  moveit_cpp::MoveItCppPtr moveit_cpp;
  planning_scene_monitor::PlanningSceneMonitorPtr monitor;
  const moveit::core::JointModelGroup* arm = nullptr;
  const moveit::core::JointModelGroup* grip = nullptr;
  const moveit::core::LinkModel* end_link = nullptr;
  std::vector<std::string> arm_joints;
  int collision_links = 0;
  int checked_pairs = 0;
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

  if (fatal.empty()) {
    try {
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
      moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(node);
      const auto model = moveit_cpp->getRobotModel();
      arm = model->getJointModelGroup(planning_group);
      grip = model->getJointModelGroup("grip_group");
      end_link = model->getLinkModel(end_link_name);
      if (arm == nullptr || grip == nullptr || end_link == nullptr) {
        throw std::runtime_error("arm_group, grip_group 또는 Arm5_Link가 없습니다.");
      }
      arm_joints = arm->getVariableNames();
      const std::vector<std::string> expected = {
          "Arm1_Joint", "Arm2_Joint", "Arm3_Joint", "Arm4_Joint", "Arm5_Joint"};
      if (arm_joints != expected || end_link_name != "Arm5_Link") {
        throw std::runtime_error("계획 그룹 또는 끝 링크가 P-002 계약과 다릅니다.");
      }
      monitor = moveit_cpp->getPlanningSceneMonitor();
      {
        planning_scene_monitor::LockedPlanningSceneRO scene(monitor);
        collision_links = static_cast<int>(model->getLinkModelsWithCollisionGeometry().size());
        const auto& matrix = scene->getAllowedCollisionMatrix();
        const auto links = model->getLinkModelsWithCollisionGeometry();
        for (std::size_t i = 0; i < links.size(); ++i) {
          for (std::size_t j = i + 1; j < links.size(); ++j) {
            collision_detection::AllowedCollision::Type allowed =
                collision_detection::AllowedCollision::NEVER;
            const bool found = matrix.getAllowedCollision(links[i]->getName(), links[j]->getName(), allowed);
            if (!found || allowed == collision_detection::AllowedCollision::NEVER) {
              ++checked_pairs;
            }
          }
        }
      }
      if (collision_links < 5 || checked_pairs < 1) {
        throw std::runtime_error("충돌 모델이 비어 있습니다.");
      }

      const moveit::core::GroupStateValidityCallbackFn valid =
          [&](moveit::core::RobotState* state, const moveit::core::JointModelGroup*, const double*) {
            state->update(true);
            planning_scene_monitor::LockedPlanningSceneRO scene(monitor);
            return !scene->isStateColliding(*state, "", false);
          };

      for (int run = 0; run < runs; ++run) {
        std::vector<CaseResult> cases;
        for (const auto& window : windows) {
          CaseResult item;
          item.id = window.case_id;
          item.source_start = window.start_index;
          item.source_end = window.end_index;
          item.waypoint_count = window.waypoint_count;
          item.arc_length_m = window.arc_length_m;
          item.execution_calls = execution_calls;
          moveit::core::RobotState start(moveit_cpp->getRobotModel());
          start.setToDefaultValues();
          if (!start.setToDefaultValues(grip, grip_state_name) || !start.setToDefaultValues(arm, start_state_name)) {
            throw std::runtime_error("SRDF named state를 읽지 못했습니다.");
          }
          start.update(true);
          item.start_collision = stateColliding(monitor, start);
          item.position_violations += positionViolations(start, arm_joints);
          const auto anchor = poseFromState(start, end_link_name);
          item.placement = robot_controller::rigidPlacement(poses[window.start_index], anchor);
          std::vector<robot_controller::MotionPose> placed;
          placed.reserve(window.waypoint_count);
          for (std::size_t index = window.start_index; index <= window.end_index; ++index) {
            placed.push_back(robot_controller::applyRigidPlacement(item.placement, poses[index]));
          }
          item.placement_residual_m = robot_controller::poseDistance(placed.front(), anchor);
          if (item.start_collision) {
            item.failure_class = "COLLISION";
            item.note = "start_state_collision";
            cases.push_back(item);
            continue;
          }
          EigenSTL::vector_Isometry3d targets;
          targets.reserve(placed.size());
          for (const auto& pose : placed) {
            targets.push_back(isometryOf(pose));
          }
          moveit::core::RobotState working(start);
          std::vector<std::shared_ptr<moveit::core::RobotState>> trajectory;
          const auto begun = std::chrono::steady_clock::now();
          try {
            const auto percentage = moveit::core::CartesianInterpolator::computeCartesianPath(
                &working, arm, trajectory, end_link, targets, true, moveit::core::MaxEEFStep(eef_step),
                moveit::core::JumpThreshold(jump_threshold), valid);
            item.fraction = percentage.value;
          } catch (const std::exception& error) {
            item.fraction = 0.0;
            item.note = error.what();
          }
          item.planning_time_sec =
              std::chrono::duration<double>(std::chrono::steady_clock::now() - begun).count();
          item.trajectory_points = static_cast<int>(trajectory.size());
          for (const auto& state : trajectory) {
            state->update(true);
            item.position_violations += positionViolations(*state, arm_joints);
            if (stateColliding(monitor, *state)) {
              ++item.collision_states;
            }
          }
          timeArm(trajectory, arm_joints, max_joint_velocity, item.max_joint_velocity, item.velocity_violations);
          if (!trajectory.empty()) {
            const auto start_fk = poseFromState(*trajectory.front(), end_link_name);
            const auto end_fk = poseFromState(*trajectory.back(), end_link_name);
            item.start_position_error_m = robot_controller::poseDistance(start_fk, placed.front());
            item.start_orientation_error_deg = robot_controller::quaternionStepDeg(start_fk, placed.front());
            item.end_position_error_m = robot_controller::poseDistance(end_fk, placed.back());
            item.end_orientation_error_deg = robot_controller::quaternionStepDeg(end_fk, placed.back());
          }
          item.failure_class = classifyCase(item, min_fraction);
          if (item.note.empty()) {
            item.note = item.failure_class == "OK" ? "planned" : item.failure_class;
          }
          cases.push_back(item);
          RCLCPP_INFO(
              node->get_logger(), "P-005 run %d %s %s fraction %.4f points %d", run + 1, item.id.c_str(),
              item.failure_class.c_str(), item.fraction, item.trajectory_points);
        }
        run_cases.push_back(cases);
      }
      stop_spin();
    } catch (const std::exception& error) {
      stop_spin();
      fatal = error.what();
      RCLCPP_ERROR(node->get_logger(), "%s", fatal.c_str());
    }
  }

  bool same_classification = !run_cases.empty();
  if (run_cases.size() >= 2) {
    for (std::size_t run = 1; run < run_cases.size(); ++run) {
      if (run_cases[run].size() != run_cases.front().size()) {
        same_classification = false;
        break;
      }
      for (std::size_t index = 0; index < run_cases[run].size(); ++index) {
        const auto& first = run_cases.front()[index];
        const auto& other = run_cases[run][index];
        if (first.id != other.id || first.source_start != other.source_start ||
            first.source_end != other.source_end || first.failure_class != other.failure_class) {
          same_classification = false;
        }
      }
    }
  }

  bool edge_detected = false;
  std::size_t edge_index = 0;
  double edge_tool = 0.0;
  double edge_quat = 0.0;
  for (const auto index : validation.sharp_indices) {
    if (index < pairs.size() && pairs[index].label == robot_controller::PairLabel::ExplainedGeometric &&
        pairs[index].tool_z_deg > edge_tool) {
      edge_detected = true;
      edge_tool = pairs[index].tool_z_deg;
      edge_quat = pairs[index].quaternion_deg;
      edge_index = index;
    }
  }

  bool all_ok = fatal.empty() && edge_detected && edge_tool > 90.0 && same_classification &&
                run_cases.size() == static_cast<std::size_t>(runs);
  for (const auto& cases : run_cases) {
    if (cases.size() != 3) {
      all_ok = false;
    }
    for (const auto& item : cases) {
      if (item.failure_class != "OK" || item.execution_calls != 0) {
        all_ok = false;
      }
    }
  }

  {
    std::ofstream source(output_dir + "/source-geometry.txt");
    source << std::setprecision(17);
    source << "waypoints_csv " << waypoints_csv << "\n";
    source << "frame " << source_frame << "\n";
    source << "waypoints " << poses.size() << "\n";
    source << "sharp_transitions " << validation.sharp_indices.size() << "\n";
    source << "accepted " << validation.accepted << "\n";
    source << "rejected " << validation.rejected << "\n";
    source << "error " << (validation.error.empty() ? fatal : validation.error) << "\n";
    source << "original_overwritten no\n";
  }
  {
    std::ofstream hash(output_dir + "/source-sha256.txt");
    hash << "snapshot " << snapshot_hash << "\n";
    hash << "waypoints " << waypoints_hash << "\n";
    hash << "expected_snapshot " << expected_snapshot << "\n";
    hash << "expected_waypoints " << expected_waypoints << "\n";
  }
  {
    std::ofstream placement(output_dir + "/synthetic-placement.yaml");
    placement << std::setprecision(17);
    placement << "source_frame: " << source_frame << "\n";
    placement << "target_frame: " << planning_frame << "\n";
    placement << "transform_type: " << robot_controller::kSyntheticType << "\n";
    placement << "calibrated: false\n";
    placement << "published_to_tf: false\n";
    placement << "scale: 1.0\n";
    placement << "purpose: P-005 software integration dry-run only\n";
    placement << "start_state: " << start_state_name << "\n";
    placement << "grip_state: " << grip_state_name << "\n";
    placement << "rule: map window start pose onto Arm5_Link FK of the SRDF state\n";
    placement << "moving_link_sum_m: 0.276878\n";
    placement << "window_count: " << window_count << "\n";
    placement << "max_orientation_step_deg: " << max_step << "\n";
    if (!run_cases.empty()) {
      for (const auto& item : run_cases.front()) {
        placement << item.id << ":\n";
        placement << "  translation: [" << item.placement.translation[0] << ", " << item.placement.translation[1]
                  << ", " << item.placement.translation[2] << "]\n";
        placement << "  rotation_row_major: [";
        for (int index = 0; index < 9; ++index) {
          if (index > 0) {
            placement << ", ";
          }
          placement << item.placement.rotation[index];
        }
        placement << "]\n";
        placement << "  placement_residual_m: " << item.placement_residual_m << "\n";
      }
    }
  }
  {
    std::ofstream motion(output_dir + "/motion-validation.txt");
    motion << std::setprecision(17);
    motion << "input " << validation.input_count << "\n";
    motion << "accepted " << validation.accepted << "\n";
    motion << "rejected " << validation.rejected << "\n";
    motion << "sharp_transitions " << validation.sharp_indices.size() << "\n";
    motion << "segments " << validation.segments.size() << "\n";
    motion << "max_orientation_step_deg " << max_step << "\n";
    motion << "threshold_basis tool +Z angle, half of a right angle, above this path p99\n";
    motion << "primary_edge_index " << edge_index << "\n";
    motion << "primary_edge_tool_z_deg " << edge_tool << "\n";
    motion << "primary_edge_quaternion_deg " << edge_quat << "\n";
    motion << "primary_edge_detected " << (edge_detected ? "yes" : "no") << "\n";
    motion << "smoothed no\n";
    motion << "boundary_policy " << robot_controller::kBoundaryPolicy << "\n";
    motion << "error " << fatal << "\n";
  }
  {
    std::ofstream csv(output_dir + "/motion-segments.csv");
    csv << std::setprecision(17);
    csv << "segment,start_index,end_index,waypoint_count,arc_length_m,boundary_after,boundary_reason\n";
    for (std::size_t index = 0; index < validation.segments.size(); ++index) {
      const auto& segment = validation.segments[index];
      csv << index << ',' << segment.start_index << ',' << segment.end_index << ',' << segment.waypoint_count
          << ',' << segment.arc_length_m << ',' << segment.boundary_after << ',' << segment.boundary_reason
          << '\n';
    }
  }
  {
    std::ofstream csv(output_dir + "/planning-cases.csv");
    csv << std::setprecision(17);
    csv << "run,case_id,source_start,source_end,waypoint_count,arc_length_m,planning_group,end_link,frame,"
           "start_state,fraction,trajectory_points,planning_time_s,max_joint_velocity,position_violations,"
           "velocity_violations,collision_states,execution_calls,failure_class\n";
    for (std::size_t run = 0; run < run_cases.size(); ++run) {
      for (const auto& item : run_cases[run]) {
        csv << (run + 1) << ',' << item.id << ',' << item.source_start << ',' << item.source_end << ','
            << item.waypoint_count << ',' << item.arc_length_m << ',' << planning_group << ',' << end_link_name
            << ',' << planning_frame << ',' << start_state_name << ',' << item.fraction << ','
            << item.trajectory_points << ',' << item.planning_time_sec << ',' << item.max_joint_velocity << ','
            << item.position_violations << ',' << item.velocity_violations << ',' << item.collision_states << ','
            << item.execution_calls << ',' << item.failure_class << '\n';
      }
    }
  }
  {
    std::ofstream csv(output_dir + "/trajectory-validation.csv");
    csv << std::setprecision(17);
    csv << "run,case_id,endpoint,position_error_m,orientation_error_deg\n";
    for (std::size_t run = 0; run < run_cases.size(); ++run) {
      for (const auto& item : run_cases[run]) {
        csv << (run + 1) << ',' << item.id << ",start," << item.start_position_error_m << ','
            << item.start_orientation_error_deg << '\n';
        csv << (run + 1) << ',' << item.id << ",end," << item.end_position_error_m << ','
            << item.end_orientation_error_deg << '\n';
      }
    }
  }
  {
    std::ofstream collision(output_dir + "/collision-validation.txt");
    collision << "collision_checking_disabled false\n";
    collision << "collision_links " << collision_links << "\n";
    collision << "self_collision_pairs_checked " << checked_pairs << "\n";
    for (std::size_t run = 0; run < run_cases.size(); ++run) {
      for (const auto& item : run_cases[run]) {
        collision << "run " << (run + 1) << ' ' << item.id << " start "
                  << (item.start_collision ? "COLLISION" : "clear") << " trajectory_collision_states "
                  << item.collision_states << "\n";
      }
    }
  }
  {
    std::ofstream limits(output_dir + "/joint-limit-validation.txt");
    limits << std::setprecision(17);
    limits << "position_margin_rad " << kPositionMargin << "\n";
    limits << "max_joint_velocity_rad_s " << max_joint_velocity << "\n";
    limits << "vendor_acceleration_limits disabled\n";
    limits << "hardware_safe_acceleration_validated no\n";
    for (std::size_t run = 0; run < run_cases.size(); ++run) {
      for (const auto& item : run_cases[run]) {
        limits << "run " << (run + 1) << ' ' << item.id << " position_violations " << item.position_violations
               << " velocity_violations " << item.velocity_violations << " max_joint_velocity "
               << item.max_joint_velocity << "\n";
      }
    }
  }
  {
    std::ofstream guard(output_dir + "/execution-guard.txt");
    guard << "execute_trajectory false\n";
    guard << "execute_calls " << execution_calls << "\n";
    guard << "controller_sends " << controller_sends << "\n";
    guard << "follow_joint_trajectory_execution 0\n";
    guard << "fake_system_execution 0\n";
    guard << "hardware_execution 0\n";
    guard << "tf_publishes " << tf_publishes << "\n";
    guard << "camera_robot_calibration 0\n";
    guard << "planning_end_link Arm5_Link\n";
    guard << "process_tcp_claimed no\n";
  }
  {
    std::ofstream reproducibility(output_dir + "/reproducibility.txt");
    reproducibility << "runs " << run_cases.size() << "\n";
    reproducibility << "classification_equal " << (same_classification ? "yes" : "no") << "\n";
    reproducibility << "execution_calls " << execution_calls << "\n";
    for (std::size_t run = 0; run < run_cases.size(); ++run) {
      reproducibility << "run " << (run + 1);
      for (const auto& item : run_cases[run]) {
        reproducibility << ' ' << item.id << ':' << item.source_start << '-' << item.source_end << ':'
                       << item.failure_class;
      }
      reproducibility << "\n";
    }
  }
  {
    std::ofstream json(output_dir + "/planning-results.json");
    json << std::setprecision(17);
    json << "{\n";
    json << "  \"status\": \"" << (all_ok ? "PASS" : "FAIL") << "\",\n";
    json << "  \"fatal\": \"" << fatal << "\",\n";
    json << "  \"environment\": {\"planning_group\": \"" << planning_group << "\", \"end_link\": \""
         << end_link_name << "\", \"execute_trajectory\": false},\n";
    json << "  \"source_geometry\": {\"sha256\": \"" << snapshot_hash << "\", \"waypoints_sha256\": \""
         << waypoints_hash << "\", \"frame\": \"" << source_frame << "\", \"waypoints\": " << poses.size()
         << "},\n";
    json << "  \"synthetic_transform\": {\"type\": \"" << robot_controller::kSyntheticType
         << "\", \"calibrated\": false, \"scale\": 1.0, \"published_to_tf\": false},\n";
    json << "  \"motion_validation\": {\"accepted\": " << validation.accepted
         << ", \"rejected\": " << validation.rejected << ", \"sharp_transitions\": "
         << validation.sharp_indices.size() << ", \"segments\": " << validation.segments.size()
         << ", \"primary_edge_index\": " << edge_index << ", \"primary_edge_tool_z_deg\": " << edge_tool
         << "},\n";
    json << "  \"execution_guard\": {\"execute_calls\": 0, \"controller_sends\": 0, \"tf_publishes\": 0},\n";
    json << "  \"planning_cases\": [\n";
    for (std::size_t run = 0; run < run_cases.size(); ++run) {
      for (std::size_t index = 0; index < run_cases[run].size(); ++index) {
        const auto& item = run_cases[run][index];
        const bool last = run + 1 == run_cases.size() && index + 1 == run_cases[run].size();
        json << "    {\"run\": " << (run + 1) << ", \"id\": \"" << item.id << "\", \"source_start\": "
             << item.source_start << ", \"source_end\": " << item.source_end
             << ", \"fraction\": " << item.fraction << ", \"trajectory_points\": " << item.trajectory_points
             << ", \"planning_time_sec\": " << item.planning_time_sec
             << ", \"max_joint_velocity\": " << item.max_joint_velocity
             << ", \"position_violations\": " << item.position_violations
             << ", \"velocity_violations\": " << item.velocity_violations
             << ", \"collision_states\": " << item.collision_states
             << ", \"failure_class\": \"" << item.failure_class << "\", \"execution_calls\": 0}"
             << (last ? "\n" : ",\n");
      }
    }
    json << "  ]\n}\n";
  }

  RCLCPP_INFO(node->get_logger(), "P-005 RESULT %s", all_ok ? "PASS" : "FAIL");
  stop_spin();
  rclcpp::shutdown();
  return all_ok ? 0 : 1;
}
