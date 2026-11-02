// P-022-AB 진단. P-022 기준선 파일을 쓰지 않는다.
// 시작 IK timeout은 0.005 s다. group default IK timeout은 덮어쓰지 않는다.
// 궤적 파일은 쓰지 않는다. 실물 로봇에는 연결하지 않는다.

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit/robot_state/robot_state.hpp>

namespace
{
struct Pose
{
  Eigen::Isometry3d pose;
};

struct Run
{
  std::vector<Pose> poses;
};

std::string read_file(const std::string & path)
{
  std::ifstream input(path);
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

std::vector<Run> read_runs(const std::string & path)
{
  std::ifstream input(path);
  std::vector<Run> runs;
  std::string tag;
  while (input >> tag)
  {
    if (tag != "RUN")
    {
      throw std::runtime_error("poses file");
    }
    int run_id = 0;
    int count = 0;
    input >> run_id >> count;
    Run run;
    for (int index = 0; index < count; ++index)
    {
      double x, y, z, qx, qy, qz, qw;
      input >> x >> y >> z >> qx >> qy >> qz >> qw;
      Pose pose;
      pose.pose = Eigen::Isometry3d::Identity();
      pose.pose.translation() = Eigen::Vector3d(x, y, z);
      pose.pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
      run.poses.push_back(pose);
    }
    if (run_id != static_cast<int>(runs.size()))
    {
      throw std::runtime_error("run id");
    }
    runs.push_back(run);
  }
  return runs;
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

void write_vector(std::ostream & output, const std::vector<double> & values)
{
  output << "[";
  for (std::size_t index = 0; index < values.size(); ++index)
  {
    if (index)
    {
      output << ", ";
    }
    write_number(output, values[index]);
  }
  output << "]";
}

bool matches_baseline_partial(double fraction, int pose_count)
{
  if (pose_count != 34 || !std::isfinite(fraction))
  {
    return false;
  }
  return std::abs(fraction * 33.0 - 20.0) < 1e-4;
}

struct StartResult
{
  bool success = false;
  double elapsed_s = 0;
  std::vector<double> joints;
  double position_error_m = std::numeric_limits<double>::quiet_NaN();
  double orientation_error_rad = std::numeric_limits<double>::quiet_NaN();
  bool within_limits = false;
  bool fk_within_precision = false;
};

StartResult solve_start(moveit::core::RobotState & state, const moveit::core::JointModelGroup * group,
                        const moveit::core::LinkModel * tool, const Eigen::Isometry3d & target, double timeout)
{
  StartResult result;
  state.setToDefaultValues();
  state.update();
  const auto started = std::chrono::steady_clock::now();
  result.success = state.setFromIK(group, target, std::string("tool0"), timeout);
  result.elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  if (!result.success)
  {
    return result;
  }
  state.update();
  state.copyJointGroupPositions(group, result.joints);
  const Eigen::Isometry3d actual = state.getGlobalLinkTransform(tool);
  result.position_error_m = (actual.translation() - target.translation()).norm();
  result.orientation_error_rad =
    Eigen::Quaterniond(actual.linear()).angularDistance(Eigen::Quaterniond(target.linear()));
  result.within_limits = true;
  for (const moveit::core::JointModel * joint : group->getActiveJointModels())
  {
    if (!state.satisfiesBounds(joint))
    {
      result.within_limits = false;
    }
  }
  const moveit::core::CartesianPrecision precision;
  result.fk_within_precision =
    result.position_error_m <= precision.translational && result.orientation_error_rad <= precision.rotational;
  return result;
}

void write_start(std::ostream & output, const StartResult & start)
{
  output << ", \"start_ik_success\": " << (start.success ? "true" : "false")
         << ", \"start_ik_elapsed_s\": ";
  write_number(output, start.elapsed_s);
  output << ", \"joint_state\": ";
  if (start.joints.empty())
  {
    output << "null";
  }
  else
  {
    write_vector(output, start.joints);
  }
  output << ", \"position_error_m\": ";
  write_number(output, start.position_error_m);
  output << ", \"orientation_error_rad\": ";
  write_number(output, start.orientation_error_rad);
  output << ", \"within_urdf_limits\": " << (start.success && start.within_limits ? "true" : "false")
         << ", \"fk_within_cartesian_precision\": "
         << (start.success && start.fk_within_precision ? "true" : "false");
}

void write_target(std::ostream & output, const Eigen::Isometry3d & pose)
{
  const Eigen::Quaterniond rotation(pose.linear());
  output << ", \"target_position_m\": [";
  write_number(output, pose.translation().x());
  output << ", ";
  write_number(output, pose.translation().y());
  output << ", ";
  write_number(output, pose.translation().z());
  output << "], \"target_quaternion_xyzw\": [";
  write_number(output, rotation.x());
  output << ", ";
  write_number(output, rotation.y());
  output << ", ";
  write_number(output, rotation.z());
  output << ", ";
  write_number(output, rotation.w());
  output << "]";
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 6)
  {
    std::cerr << "usage: p022ab a|b urdf srdf runs output\n";
    return 2;
  }
  const std::string mode(argv[1]);
  if (mode != "a" && mode != "b")
  {
    std::cerr << "usage: p022ab a|b urdf srdf runs output\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("p022ab");
  auto urdf = urdf::parseURDF(read_file(argv[2]));
  auto srdf = std::make_shared<srdf::Model>();
  if (!urdf || !srdf->initString(*urdf, read_file(argv[3])))
  {
    std::cerr << "FAIL_ROBOT_MODEL_LOAD\n";
    return 3;
  }
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  auto * group = model->getJointModelGroup("ur_manipulator");
  const auto * tool = model->getLinkModel("tool0");
  auto solver = std::make_shared<kdl_kinematics_plugin::KDLKinematicsPlugin>();
  if (group == nullptr || tool == nullptr ||
      !solver->initialize(node, *model, "ur_manipulator", "base_link", {"tool0"}, 0.005))
  {
    std::cerr << "FAIL_ROBOT_MODEL_LOAD\n";
    return 3;
  }
  model->setKinematicsAllocators(
    {{"ur_manipulator", [solver](const moveit::core::JointModelGroup *) { return solver; }}});

  const double start_timeout = 0.005;
  const double group_timeout = group->getDefaultIKTimeout();
  std::cout << "DEFAULT_GROUP_IK_TIMEOUT_S " << group_timeout << "\n";
  std::cout << "SET_DEFAULT_IK_TIMEOUT false\n";
  std::cout << "RESEED_DISABLE_AT_0_005S NOT_AVAILABLE\n";
  std::cout << "RANDOM_RESEED_SEED_CONTROL NOT_AVAILABLE\n" << std::flush;

  moveit::core::RobotState seed_state(model);
  seed_state.setToDefaultValues();
  std::vector<double> default_joints;
  seed_state.copyJointGroupPositions(group, default_joints);
  const auto joint_names = group->getActiveJointModelNames();

  const auto runs = read_runs(argv[4]);
  const int repeats = mode == "a" ? 5 : 20;
  const moveit::core::MaxEEFStep step(0.005, 0.0);
  const moveit::core::CartesianPrecision precision;
  int baseline_partial_count = 0;

  std::ofstream output(argv[5]);
  output << std::setprecision(17);
  output << "{\n  \"gate\": \"P-022-AB\",\n  \"experiment\": \"" << (mode == "a" ? "A" : "B") << "\",\n"
         << "  \"seed_policy\": \"SET_TO_DEFAULT_VALUES\",\n"
         << "  \"first_ik_seed_control\": \"ROBOT_STATE_JOINTS_BEFORE_SET_FROM_IK\",\n"
         << "  \"reseed_policy\": \"STOCK_SEARCH_POSITION_IK\",\n"
         << "  \"reseed_disable_at_0_005s\": \"NOT_AVAILABLE\",\n"
         << "  \"random_reseed_seed_control\": \"NOT_AVAILABLE\",\n"
         << "  \"set_default_ik_timeout\": false,\n  \"start_ik_timeout_s\": ";
  write_number(output, start_timeout);
  output << ",\n  \"default_group_ik_timeout_s\": ";
  write_number(output, group_timeout);
  output << ",\n  \"eef_step_m\": 0.005,\n  \"eef_rotation_step_rad\": 0,\n"
         << "  \"translational_precision_m\": ";
  write_number(output, precision.translational);
  output << ",\n  \"rotational_precision_rad\": ";
  write_number(output, precision.rotational);
  output << ",\n  \"repeat_count\": " << repeats << ",\n  \"trajectory_written\": false,\n  \"joint_names\": [";
  for (std::size_t index = 0; index < joint_names.size(); ++index)
  {
    if (index)
    {
      output << ", ";
    }
    output << "\"" << joint_names[index] << "\"";
  }
  output << "],\n  \"first_ik_seed_joints\": ";
  write_vector(output, default_joints);
  output << ",\n  \"repeats\": [\n";

  for (int repeat = 0; repeat < repeats; ++repeat)
  {
    if (repeat)
    {
      output << ",\n";
    }
    output << "    {\"repeat\": " << repeat << ", \"runs\": [\n";
    bool first = true;
    const std::size_t begin = mode == "b" ? 20 : 0;
    const std::size_t end = mode == "b" ? 21 : runs.size();
    if (mode == "b" && runs.size() < 21)
    {
      throw std::runtime_error("run 20 missing");
    }
    for (std::size_t index = begin; index < end; ++index)
    {
      const auto & run = runs[index];
      if (!first)
      {
        output << ",\n";
      }
      first = false;
      output << "      {\"run_id\": " << index << ", \"waypoint_count\": " << run.poses.size()
             << ", \"timeout_s\": ";
      write_number(output, start_timeout);
      output << ", \"seed_policy\": \"SET_TO_DEFAULT_VALUES\"";
      if (run.poses.size() < 2)
      {
        output << ", \"api_status\": \"FAIL_PLANNING_REPRESENTATION\", \"reason\": \"TOO_FEW_POSES\""
               << ", \"start_ik_success\": false, \"cartesian_fraction\": null}";
        continue;
      }
      write_target(output, run.poses.front().pose);
      moveit::core::RobotState state(model);
      const StartResult start = solve_start(state, group, tool, run.poses.front().pose, start_timeout);
      write_start(output, start);
      if (!start.success)
      {
        output << ", \"api_status\": \"FAIL_CARTESIAN_API\", \"reason\": \"START_IK_FAILED\""
               << ", \"cartesian_fraction\": null, \"cartesian_elapsed_s\": null}";
        std::cout << "REPEAT " << repeat << " RUN " << index << " START_IK_FAILED\n" << std::flush;
        continue;
      }
      EigenSTL::vector_Isometry3d waypoints;
      waypoints.reserve(run.poses.size() - 1);
      for (std::size_t pose_index = 1; pose_index < run.poses.size(); ++pose_index)
      {
        waypoints.push_back(run.poses[pose_index].pose);
      }
      std::vector<std::shared_ptr<moveit::core::RobotState>> trajectory;
      const auto path_started = std::chrono::steady_clock::now();
      moveit::core::CartesianInterpolator::Percentage fraction =
        moveit::core::CartesianInterpolator::computeCartesianPath(
          &state, group, trajectory, tool, waypoints, true, step, precision);
      const double path_elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - path_started).count();
      const double fraction_value = fraction.value;
      const bool partial = matches_baseline_partial(fraction_value, static_cast<int>(run.poses.size()));
      if (partial)
      {
        ++baseline_partial_count;
      }
      output << ", \"api_status\": \"OK\", \"reason\": null, \"cartesian_fraction\": ";
      write_number(output, fraction_value);
      output << ", \"cartesian_elapsed_s\": ";
      write_number(output, path_elapsed);
      output << ", \"matches_20_over_33\": " << (partial ? "true" : "false")
             << ", \"trajectory_written\": false}";
      trajectory.clear();
      std::cout << "REPEAT " << repeat << " RUN " << index << " FRACTION " << std::setprecision(17) << fraction_value
                << "\n"
                << std::flush;
    }
    output << "\n    ]}";
  }
  output << "\n  ],\n  \"baseline_20_over_33_count\": " << baseline_partial_count << "\n}\n";
  std::cout << "BASELINE_20_OVER_33_COUNT " << baseline_partial_count << "\n";
  rclcpp::shutdown();
  return 0;
}
