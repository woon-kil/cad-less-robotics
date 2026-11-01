// pick_ik qualification. 패키지 기본 파라미터만 쓴다.
// P-022 pose와 fraction은 읽지 않는다.

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <random_numbers/random_numbers.h>

namespace
{
std::string read_file(const std::string & path)
{
  std::ifstream input(path);
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

geometry_msgs::msg::Pose to_msg(const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.translation().x();
  msg.position.y = pose.translation().y();
  msg.position.z = pose.translation().z();
  const Eigen::Quaterniond rotation(pose.linear());
  msg.orientation.x = rotation.x();
  msg.orientation.y = rotation.y();
  msg.orientation.z = rotation.z();
  msg.orientation.w = rotation.w();
  return msg;
}

double position_error(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  return (a.translation() - b.translation()).norm();
}

double orientation_error(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  return Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle();
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3)
  {
    std::cerr << "usage: pick_ik_qualify urdf srdf\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pick_ik_qualify");

  auto urdf = urdf::parseURDF(read_file(argv[1]));
  auto srdf = std::make_shared<srdf::Model>();
  if (!urdf || !srdf->initString(*urdf, read_file(argv[2])))
  {
    std::cerr << "FAIL_ROBOT_MODEL_LOAD\n";
    return 3;
  }
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  auto * group = model->getJointModelGroup("ur_manipulator");
  if (group == nullptr || model->getLinkModel("tool0") == nullptr)
  {
    std::cerr << "FAIL_ROBOT_MODEL_LOAD group\n";
    return 3;
  }

  pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
    "moveit_core", "kinematics::KinematicsBase");
  std::shared_ptr<kinematics::KinematicsBase> solver;
  try
  {
    solver = loader.createSharedInstance("pick_ik/PickIkPlugin");
  }
  catch (const std::exception & error)
  {
    std::cerr << "FAIL_PLUGIN_LOAD " << error.what() << "\n";
    return 4;
  }
  // UR kinematics.yaml의 search resolution. pick_ik 파라미터는 선언하지 않는다.
  const double search_discretization = 0.005;
  if (!solver->initialize(node, *model, "ur_manipulator", "base_link", {"tool0"}, search_discretization))
  {
    std::cerr << "FAIL_SOLVER_INITIALIZE\n";
    return 4;
  }
  std::cout << "PLUGIN_CLASS pick_ik/PickIkPlugin\n";
  std::cout << "PARAMETER_OVERRIDE NONE\n";
  auto listed = node->list_parameters({}, 10);
  for (const auto & name : listed.names)
  {
    std::cout << "PARAM " << name << " " << node->get_parameter(name).value_to_string() << "\n";
  }

  // UR kinematics.yaml kinematics_solver_timeout. 결과를 보고 바꾸지 않는다.
  const double ik_timeout = 0.005;
  const int sample_count = 20;
  const int repeat_count = 3;
  random_numbers::RandomNumberGenerator rng(20261101);
  int success_count = 0;
  int call_count = 0;
  int limit_violation = 0;
  int repeat_success_mismatch = 0;
  double max_position_error = 0.0;
  double max_orientation_error = 0.0;

  for (int sample = 0; sample < sample_count; ++sample)
  {
    moveit::core::RobotState state(model);
    state.setToRandomPositions(group, rng);
    state.updateLinkTransforms();
    const Eigen::Isometry3d base = state.getGlobalLinkTransform("base_link");
    const Eigen::Isometry3d target = base.inverse() * state.getGlobalLinkTransform("tool0");
    std::vector<double> seed;
    state.copyJointGroupPositions(group, seed);
    std::vector<int> successes;
    std::vector<std::vector<double>> solutions;
    for (int repeat = 0; repeat < repeat_count; ++repeat)
    {
      std::vector<double> solution;
      moveit_msgs::msg::MoveItErrorCodes error;
      const bool ok = solver->searchPositionIK(to_msg(target), seed, ik_timeout, solution, error);
      ++call_count;
      successes.push_back(ok ? 1 : 0);
      if (!ok)
      {
        std::cout << "SAMPLE " << sample << " REPEAT " << repeat << " IK_SUCCESS 0 ERROR " << error.val << "\n";
        continue;
      }
      ++success_count;
      solutions.push_back(solution);
      moveit::core::RobotState solved(model);
      solved.setToDefaultValues();
      solved.setJointGroupPositions(group, solution);
      const bool inside = solved.satisfiesBounds(group);
      if (!inside)
      {
        ++limit_violation;
      }
      solved.updateLinkTransforms();
      const Eigen::Isometry3d base_solved = solved.getGlobalLinkTransform("base_link");
      const Eigen::Isometry3d reached = base_solved.inverse() * solved.getGlobalLinkTransform("tool0");
      const double pos = position_error(target, reached);
      const double ori = orientation_error(target, reached);
      max_position_error = std::max(max_position_error, pos);
      max_orientation_error = std::max(max_orientation_error, ori);
      double joint_delta = 0.0;
      for (std::size_t joint = 0; joint < seed.size(); ++joint)
      {
        joint_delta = std::max(joint_delta, std::abs(solution[joint] - seed[joint]));
      }
      std::cout << std::setprecision(17);
      std::cout << "SAMPLE " << sample << " REPEAT " << repeat
                << " IK_SUCCESS 1 LIMIT_OK " << (inside ? 1 : 0)
                << " POSITION_ERROR_M " << pos
                << " ORIENTATION_ERROR_RAD " << ori
                << " MAX_JOINT_DELTA_RAD " << joint_delta << "\n";
    }
    const bool same_success = successes[0] == successes[1] && successes[1] == successes[2];
    if (!same_success)
    {
      ++repeat_success_mismatch;
    }
  }

  std::cout << "SAMPLE_COUNT " << sample_count << "\n";
  std::cout << "REPEAT_COUNT " << repeat_count << "\n";
  std::cout << "IK_CALL_COUNT " << call_count << "\n";
  std::cout << "IK_SUCCESS_COUNT " << success_count << "\n";
  std::cout << "LIMIT_VIOLATION_COUNT " << limit_violation << "\n";
  std::cout << "REPEAT_SUCCESS_MISMATCH_COUNT " << repeat_success_mismatch << "\n";
  std::cout << "MAX_POSITION_ERROR_M " << max_position_error << "\n";
  std::cout << "MAX_ORIENTATION_ERROR_RAD " << max_orientation_error << "\n";
  std::cout << "Q_EQUALS_SEED_REQUIRED NO\n";
  std::cout << "BENCHMARK_EXECUTED NO\n";
  rclcpp::shutdown();
  return 0;
}
