// P-022 Cartesian planning. 동결된 eef_step과 jump, avoid_collisions만 사용한다.
// 궤적 파일은 쓰지 않는다. 실물 로봇에는 연결하지 않는다.

#include <cmath>
#include <fstream>
#include <iostream>
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
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 5 && argc != 6)
  {
    std::cerr << "usage: p022_plan urdf srdf runs output [ik_timeout_s]\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("p022_plan");

  auto urdf = urdf::parseURDF(read_file(argv[1]));
  if (!urdf)
  {
    std::cerr << "FAIL_ROBOT_MODEL_LOAD urdf\n";
    return 3;
  }
  auto srdf = std::make_shared<srdf::Model>();
  if (!srdf->initString(*urdf, read_file(argv[2])))
  {
    std::cerr << "FAIL_ROBOT_MODEL_LOAD srdf\n";
    return 3;
  }
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  auto * group = model->getJointModelGroup("ur_manipulator");
  const auto * tool = model->getLinkModel("tool0");
  if (group == nullptr || tool == nullptr)
  {
    std::cerr << "FAIL_ROBOT_MODEL_LOAD group\n";
    return 3;
  }

  auto solver = std::make_shared<kdl_kinematics_plugin::KDLKinematicsPlugin>();
  const double search_discretization = 0.005;
  const double ik_timeout = argc == 6 ? std::stod(argv[5]) : 0.005;
  if (!solver->initialize(node, *model, "ur_manipulator", "base_link", {"tool0"}, search_discretization))
  {
    std::cerr << "FAIL_ROBOT_MODEL_LOAD kinematics\n";
    return 3;
  }
  moveit::core::SolverAllocatorFn allocator =
    [solver](const moveit::core::JointModelGroup *) { return solver; };
  model->setKinematicsAllocators({{"ur_manipulator", allocator}});
  std::cout << "DEFAULT_GROUP_IK_TIMEOUT_S " << group->getDefaultIKTimeout() << "\n";
  group->setDefaultIKTimeout(ik_timeout);
  std::cout << "APPLIED_IK_TIMEOUT_S " << group->getDefaultIKTimeout() << "\n";

  moveit::core::RobotState probe(model);
  probe.setToDefaultValues();
  probe.updateLinkTransforms();
  const Eigen::Isometry3d world_to_base = probe.getGlobalLinkTransform("base_link");
  const double translation_error = world_to_base.translation().norm();
  const double rotation_error =
    (world_to_base.linear() - Eigen::Matrix3d::Identity()).norm();
  std::cout << "WORLD_TO_BASE_TRANSLATION_M " << translation_error << "\n";
  std::cout << "WORLD_TO_BASE_ROTATION_ERROR " << rotation_error << "\n";

  const auto runs = read_runs(argv[3]);
  std::ofstream output(argv[4]);
  output << "{\n  \"runs\": [\n";
  bool first = true;
  for (std::size_t index = 0; index < runs.size(); ++index)
  {
    const auto & run = runs[index];
    if (!first)
    {
      output << ",\n";
    }
    first = false;
    output << "    {\"run_id\": " << index
           << ", \"waypoint_count\": " << run.poses.size();
    if (run.poses.size() < 2)
    {
      output << ", \"api_status\": \"FAIL_PLANNING_REPRESENTATION\", \"fraction\": null}";
      continue;
    }

    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    state.update();
    if (!state.setFromIK(group, run.poses.front().pose, std::string("tool0"), ik_timeout))
    {
      output << ", \"api_status\": \"FAIL_CARTESIAN_API\", \"reason\": \"START_IK_FAILED\", \"fraction\": null}";
      continue;
    }

    EigenSTL::vector_Isometry3d waypoints;
    waypoints.reserve(run.poses.size() - 1);
    for (std::size_t pose_index = 1; pose_index < run.poses.size(); ++pose_index)
    {
      waypoints.push_back(run.poses[pose_index].pose);
    }
    std::vector<std::shared_ptr<moveit::core::RobotState>> trajectory;
    const moveit::core::MaxEEFStep step(0.005, 0.0);
    const moveit::core::CartesianPrecision precision;
    try
    {
      moveit::core::CartesianInterpolator::Percentage fraction =
        moveit::core::CartesianInterpolator::computeCartesianPath(
          &state, group, trajectory, tool, waypoints, true, step, precision);
      output << ", \"api_status\": \"OK\", \"fraction\": " << static_cast<double>(fraction)
             << ", \"trajectory_written\": false}";
    }
    catch (const std::exception & error)
    {
      output << ", \"api_status\": \"FAIL_CARTESIAN_API\", \"reason\": \"" << error.what()
             << "\", \"fraction\": null}";
    }
  }
  output << "\n  ]\n}\n";
  rclcpp::shutdown();
  return 0;
}
