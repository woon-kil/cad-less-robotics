#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <urdf/model.h>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace
{
constexpr double kPositionMargin = 1e-6;
constexpr double kVelocityMargin = 1e-4;

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

std::string jsonNumber(double value)
{
  std::ostringstream out;
  out << std::setprecision(10) << value;
  return out.str();
}

struct CaseResult
{
  std::string id;
  std::string start_name;
  std::string goal_name;
  std::map<std::string, double> start_state;
  std::map<std::string, double> goal_state;
  bool success{false};
  double planning_time_sec{0.0};
  int trajectory_points{0};
  bool start_collision{true};
  bool goal_collision{true};
  int trajectory_collision_points{0};
  int position_limit_violations{0};
  int velocity_limit_violations{0};
  bool velocity_timed{false};
  int error_code{0};
  std::string note;
};

int positionViolations(
  const moveit::core::RobotState& state,
  const std::vector<std::string>& joints)
{
  int violations = 0;
  const auto& model = *state.getRobotModel();
  for (const auto& name : joints) {
    const auto& bounds = model.getVariableBounds(name);
    if (!bounds.position_bounded_) {
      continue;
    }
    const double value = state.getVariablePosition(name);
    if (value < bounds.min_position_ - kPositionMargin ||
      value > bounds.max_position_ + kPositionMargin) {
      ++violations;
    }
  }
  return violations;
}

bool stateColliding(
  const planning_scene_monitor::PlanningSceneMonitorPtr& monitor,
  const moveit::core::RobotState& state)
{
  planning_scene_monitor::LockedPlanningSceneRO scene(monitor);
  moveit::core::RobotState copy(state);
  copy.update(true);
  return scene->isStateColliding(copy, "", false);
}

class JointStatePump
{
public:
  JointStatePump(
    const rclcpp::Node::SharedPtr& node,
    const std::vector<std::string>& names)
  : names_(names), positions_(names.size(), 0.0)
  {
    publisher_ = node->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS());
    timer_ = node->create_wall_timer(std::chrono::milliseconds(20), [this]() { publish(); });
  }

  void publish()
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = rclcpp::Clock().now();
    std::lock_guard<std::mutex> lock(mutex_);
    message.name = names_;
    message.position = positions_;
    publisher_->publish(message);
  }

private:
  std::vector<std::string> names_;
  std::vector<double> positions_;
  std::mutex mutex_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

std::map<std::string, double> armPositions(
  const moveit::core::RobotState& state,
  const std::vector<std::string>& joints)
{
  std::map<std::string, double> values;
  for (const auto& name : joints) {
    values[name] = state.getVariablePosition(name);
  }
  return values;
}

void writeResults(
  const std::string& directory,
  const std::vector<CaseResult>& cases,
  int collision_links,
  int checked_pairs,
  const std::string& group,
  const std::string& pipeline,
  const std::string& planner)
{
  int successes = 0;
  int points = 0;
  int collision_invalid = 0;
  int position_violations = 0;
  int velocity_violations = 0;
  for (const auto& item : cases) {
    successes += item.success ? 1 : 0;
    points += item.trajectory_points;
    if (item.start_collision || item.goal_collision || item.trajectory_collision_points > 0) {
      ++collision_invalid;
    }
    position_violations += item.position_limit_violations;
    velocity_violations += item.velocity_limit_violations;
  }

  std::ofstream json(directory + "/planning-results.json");
  json << std::setprecision(10);
  json << "{\n";
  json << "  \"planning_group\": \"" << group << "\",\n";
  json << "  \"planning_pipeline\": \"" << pipeline << "\",\n";
  json << "  \"planner_id\": \"" << planner << "\",\n";
  json << "  \"execute_trajectory\": false,\n";
  json << "  \"executions\": 0,\n";
  json << "  \"collision_checking_disabled\": false,\n";
  json << "  \"collision_links\": " << collision_links << ",\n";
  json << "  \"self_collision_pairs_checked\": " << checked_pairs << ",\n";
  json << "  \"position_margin_rad\": " << kPositionMargin << ",\n";
  json << "  \"velocity_margin_rad_s\": " << kVelocityMargin << ",\n";
  json << "  \"cases\": [\n";
  for (std::size_t index = 0; index < cases.size(); ++index) {
    const auto& item = cases[index];
    json << "    {\n";
    json << "      \"id\": \"" << item.id << "\",\n";
    json << "      \"start_name\": \"" << item.start_name << "\",\n";
    json << "      \"goal_name\": \"" << item.goal_name << "\",\n";
    json << "      \"success\": " << (item.success ? "true" : "false") << ",\n";
    json << "      \"planning_time_sec\": " << jsonNumber(item.planning_time_sec) << ",\n";
    json << "      \"trajectory_points\": " << item.trajectory_points << ",\n";
    json << "      \"start_collision\": " << (item.start_collision ? "true" : "false") << ",\n";
    json << "      \"goal_collision\": " << (item.goal_collision ? "true" : "false") << ",\n";
    json << "      \"trajectory_collision_points\": " << item.trajectory_collision_points << ",\n";
    json << "      \"position_limit_violations\": " << item.position_limit_violations << ",\n";
    json << "      \"velocity_limit_violations\": " << item.velocity_limit_violations << ",\n";
    json << "      \"velocity_timed\": " << (item.velocity_timed ? "true" : "false") << ",\n";
    json << "      \"error_code\": " << item.error_code << ",\n";
    json << "      \"note\": \"" << item.note << "\",\n";
    json << "      \"start_state\": {";
    bool first = true;
    for (const auto& joint : item.start_state) {
      if (!first) {
        json << ", ";
      }
      first = false;
      json << "\"" << joint.first << "\": " << jsonNumber(joint.second);
    }
    json << "},\n      \"goal_state\": {";
    first = true;
    for (const auto& joint : item.goal_state) {
      if (!first) {
        json << ", ";
      }
      first = false;
      json << "\"" << joint.first << "\": " << jsonNumber(joint.second);
    }
    json << "}\n    }" << (index + 1 == cases.size() ? "\n" : ",\n");
  }
  json << "  ],\n";
  json << "  \"summary\": {\n";
  json << "    \"planning_attempts\": " << cases.size() << ",\n";
  json << "    \"planning_successes\": " << successes << ",\n";
  json << "    \"generated_trajectories\": " << successes << ",\n";
  json << "    \"trajectory_points_total\": " << points << ",\n";
  json << "    \"collision_invalid\": " << collision_invalid << ",\n";
  json << "    \"position_limit_violations\": " << position_violations << ",\n";
  json << "    \"velocity_limit_violations\": " << velocity_violations << ",\n";
  json << "    \"executions\": 0\n";
  json << "  }\n}\n";

  std::ofstream limits(directory + "/joint-limit-validation.txt");
  limits << std::setprecision(10);
  limits << "position_margin_rad " << kPositionMargin << "\n";
  limits << "velocity_margin_rad_s " << kVelocityMargin << "\n";
  limits << "position_limit_violations " << position_violations << "\n";
  limits << "velocity_limit_violations " << velocity_violations << "\n";
  for (const auto& item : cases) {
    limits << item.id
           << " position_violations " << item.position_limit_violations
           << " velocity_violations " << item.velocity_limit_violations
           << " velocity_timed " << (item.velocity_timed ? "yes" : "no")
           << "\n";
  }

  std::ofstream collision(directory + "/collision-validation.txt");
  collision << "collision_checking_disabled false\n";
  collision << "collision_links " << collision_links << "\n";
  collision << "self_collision_pairs_checked " << checked_pairs << "\n";
  collision << "collision_invalid_cases " << collision_invalid << "\n";
  for (const auto& item : cases) {
    collision << item.id
              << " start " << (item.start_collision ? "COLLISION" : "clear")
              << " goal " << (item.goal_collision ? "COLLISION" : "clear")
              << " trajectory_collision_points " << item.trajectory_collision_points
              << "\n";
  }
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("dofbot_dry_plan", options);

  const bool execute = node->get_parameter("execute_trajectory").as_bool();
  if (execute) {
    RCLCPP_ERROR(node->get_logger(), "execute_trajectory는 false여야 합니다. 실행을 거부합니다.");
    rclcpp::shutdown();
    return 2;
  }

  const auto urdf_xml = readText(node->get_parameter("urdf_path").as_string());
  const auto srdf_xml = readText(node->get_parameter("srdf_path").as_string());
  node->declare_parameter("robot_description", urdf_xml);
  node->declare_parameter("robot_description_semantic", srdf_xml);
  const auto output_dir = node->get_parameter("output_dir").as_string();

  urdf::Model urdf_model;
  if (!urdf_model.initString(urdf_xml)) {
    RCLCPP_ERROR(node->get_logger(), "URDF를 읽지 못했습니다.");
    rclcpp::shutdown();
    return 2;
  }
  std::vector<std::string> movable;
  for (const auto& item : urdf_model.joints_) {
    const auto type = item.second->type;
    if (type == urdf::Joint::FIXED || type == urdf::Joint::UNKNOWN) {
      continue;
    }
    movable.push_back(item.first);
  }
  std::sort(movable.begin(), movable.end());

  JointStatePump pump(node, movable);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  pump.publish();

  moveit_cpp::MoveItCppPtr moveit_cpp;
  try {
    moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(node);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "MoveItCpp 초기화 실패: %s", error.what());
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 2;
  }

  const auto model = moveit_cpp->getRobotModel();
  const auto* arm = model->getJointModelGroup("arm_group");
  const auto* grip = model->getJointModelGroup("grip_group");
  if (arm == nullptr || grip == nullptr) {
    RCLCPP_ERROR(node->get_logger(), "arm_group 또는 grip_group이 SRDF에 없습니다.");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 2;
  }
  const auto arm_joints = arm->getVariableNames();
  const auto monitor = moveit_cpp->getPlanningSceneMonitor();

  int collision_links = 0;
  int checked_pairs = 0;
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(monitor);
    collision_links = static_cast<int>(model->getLinkModelsWithCollisionGeometry().size());
    const auto& acm = scene->getAllowedCollisionMatrix();
    const auto links = model->getLinkModelsWithCollisionGeometry();
    for (std::size_t i = 0; i < links.size(); ++i) {
      for (std::size_t j = i + 1; j < links.size(); ++j) {
        collision_detection::AllowedCollision::Type allowed_type =
          collision_detection::AllowedCollision::NEVER;
        const bool found = acm.getAllowedCollision(
          links[i]->getName(), links[j]->getName(), allowed_type);
        if (!found || allowed_type == collision_detection::AllowedCollision::NEVER) {
          ++checked_pairs;
        }
      }
    }
  }
  RCLCPP_INFO(
    node->get_logger(), "collision links %d self-collision pairs checked %d",
    collision_links, checked_pairs);
  if (collision_links < 5 || checked_pairs < 1) {
    RCLCPP_ERROR(node->get_logger(), "충돌 모델 또는 self-collision 검사가 비어 있습니다.");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 2;
  }

  auto make_state = [&](const std::string& arm_state_name, moveit::core::RobotState* state) {
    state->setToDefaultValues();
    if (!state->setToDefaultValues(grip, "open")) {
      throw std::runtime_error("SRDF grip_group open 상태가 없습니다.");
    }
    if (!state->setToDefaultValues(arm, arm_state_name)) {
      throw std::runtime_error("SRDF arm_group 상태가 없습니다: " + arm_state_name);
    }
    state->update(true);
  };

  const std::vector<std::tuple<std::string, std::string, std::string>> specs = {
    {"CASE-01", "init", "up"},
    {"CASE-02", "up", "down"},
    {"CASE-03", "down", "init"},
  };

  moveit_cpp::PlanningComponent planner("arm_group", moveit_cpp);
  moveit_cpp::PlanningComponent::PlanRequestParameters request;
  request.load(node);

  std::vector<CaseResult> cases;
  for (const auto& spec : specs) {
    CaseResult result;
    result.id = std::get<0>(spec);
    result.start_name = std::get<1>(spec);
    result.goal_name = std::get<2>(spec);
    try {
      moveit::core::RobotState start(model);
      moveit::core::RobotState goal(model);
      make_state(result.start_name, &start);
      make_state(result.goal_name, &goal);
      result.start_state = armPositions(start, arm_joints);
      result.goal_state = armPositions(goal, arm_joints);
      result.start_collision = stateColliding(monitor, start);
      result.goal_collision = stateColliding(monitor, goal);
      result.position_limit_violations += positionViolations(start, arm_joints);
      result.position_limit_violations += positionViolations(goal, arm_joints);

      if (result.start_collision || result.goal_collision) {
        result.note = "start_or_goal_in_collision";
        result.success = false;
      } else if (!planner.setStartState(start) || !planner.setGoal(goal)) {
        result.note = "rejected_start_or_goal";
        result.success = false;
      } else {
        const auto begun = std::chrono::steady_clock::now();
        // plan()만 호출한다. PlanningComponent::execute()는 호출하지 않는다.
        const auto solution = planner.plan(request);
        result.planning_time_sec = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - begun).count();
        result.error_code = solution.error_code.val;
        if (!solution || !solution.trajectory) {
          result.note = "plan_failed";
          result.success = false;
        } else {
          moveit_msgs::msg::RobotTrajectory message;
          solution.trajectory->getRobotTrajectoryMsg(message);
          const auto& trajectory = message.joint_trajectory;
          result.trajectory_points = static_cast<int>(trajectory.points.size());
          result.velocity_timed = result.trajectory_points >= 2;
          moveit::core::RobotState waypoint(start);
          for (std::size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
            const auto& point = trajectory.points[point_index];
            waypoint.setVariablePositions(trajectory.joint_names, point.positions);
            waypoint.update(true);
            result.position_limit_violations += positionViolations(waypoint, trajectory.joint_names);
            if (stateColliding(monitor, waypoint)) {
              ++result.trajectory_collision_points;
            }
            if (point_index == 0) {
              continue;
            }
            const double t0 = rclcpp::Duration(trajectory.points[point_index - 1].time_from_start).seconds();
            const double t1 = rclcpp::Duration(point.time_from_start).seconds();
            const double dt = t1 - t0;
            if (dt <= 0.0) {
              result.velocity_timed = false;
              ++result.velocity_limit_violations;
              continue;
            }
            for (std::size_t joint_index = 0; joint_index < trajectory.joint_names.size(); ++joint_index) {
              const auto& bounds = model->getVariableBounds(trajectory.joint_names[joint_index]);
              if (!bounds.velocity_bounded_) {
                continue;
              }
              const double velocity = std::abs(
                point.positions[joint_index] - trajectory.points[point_index - 1].positions[joint_index]) / dt;
              if (velocity > bounds.max_velocity_ + kVelocityMargin) {
                ++result.velocity_limit_violations;
              }
            }
          }
          result.success = solution.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS &&
            result.trajectory_points >= 2 &&
            result.trajectory_collision_points == 0 &&
            result.position_limit_violations == 0 &&
            result.velocity_limit_violations == 0 &&
            result.velocity_timed;
          result.note = result.success ? "planned" : "validation_failed";
        }
      }
    } catch (const std::exception& error) {
      result.note = error.what();
      result.success = false;
    }
    cases.push_back(result);
    RCLCPP_INFO(
      node->get_logger(), "P-002 %s %s time %.6f points %d",
      result.id.c_str(), result.success ? "PASS" : "FAIL",
      result.planning_time_sec, result.trajectory_points);
  }

  writeResults(output_dir, cases, collision_links, checked_pairs, "arm_group", request.planning_pipeline, request.planner_id);

  const bool pass = std::all_of(cases.begin(), cases.end(), [](const CaseResult& item) {
    return item.success;
  });
  RCLCPP_INFO(node->get_logger(), "P-002 RESULT %s", pass ? "PASS" : "FAIL");
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return pass ? 0 : 1;
}
