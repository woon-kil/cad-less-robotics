// pick_ik diagnostic benchmark. 세 fixture만 본다.
// timeout과 pick_ik 기본 파라미터는 결과를 보고 바꾸지 않는다.

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
#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

namespace
{
struct Fixture
{
  const char * name;
  std::vector<double> seed;
  std::vector<double> recorded;
  bool has_logged_target;
  Eigen::Isometry3d logged_target;
};

std::string read_file(const std::string & path)
{
  std::ifstream input(path);
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

Eigen::Isometry3d xyzq(double x, double y, double z, double qx, double qy, double qz, double qw)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, z);
  pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
  return pose;
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

Eigen::Isometry3d fk_tool(
  const moveit::core::RobotModelConstPtr & model, const moveit::core::JointModelGroup * group,
  const std::vector<double> & joints)
{
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.setJointGroupPositions(group, joints);
  state.updateLinkTransforms();
  const Eigen::Isometry3d base = state.getGlobalLinkTransform("base_link");
  return base.inverse() * state.getGlobalLinkTransform("tool0");
}

double position_error(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  return (a.translation() - b.translation()).norm();
}

double orientation_error(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  return std::abs(Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle());
}

double max_joint_delta(const std::vector<double> & a, const std::vector<double> & b)
{
  double delta = 0.0;
  for (std::size_t index = 0; index < a.size(); ++index)
  {
    delta = std::max(delta, std::abs(a[index] - b[index]));
  }
  return delta;
}

struct PrecisionSummary
{
  bool passed = false;
  bool max_resolution = false;
  bool ik_timeout = false;
  int max_depth = 0;
  int midpoint_ik_calls = 0;
  int midpoint_ik_failures = 0;
  double first_position_error = 0.0;
  double first_orientation_error = 0.0;
  bool first_checked = false;
};

bool validate_interval(
  const moveit::core::RobotModelConstPtr & model, const moveit::core::JointModelGroup * group,
  const moveit::core::RobotState & start, const moveit::core::RobotState & end,
  const Eigen::Isometry3d & start_pose, const Eigen::Isometry3d & end_pose, kinematics::KinematicsBase & solver,
  double ik_timeout, double width, int depth, PrecisionSummary & summary)
{
  summary.max_depth = std::max(summary.max_depth, depth);
  moveit::core::RobotState mid(model);
  start.interpolate(end, 0.5, mid);
  mid.updateLinkTransforms();
  const Eigen::Isometry3d base = mid.getGlobalLinkTransform("base_link");
  const Eigen::Isometry3d fk = base.inverse() * mid.getGlobalLinkTransform("tool0");
  Eigen::Isometry3d mid_pose(Eigen::Quaterniond(start_pose.linear()).slerp(0.5, Eigen::Quaterniond(end_pose.linear())));
  mid_pose.translation() = 0.5 * (start_pose.translation() + end_pose.translation());
  const double pos = position_error(mid_pose, fk);
  const double ori = orientation_error(mid_pose, fk);
  if (!summary.first_checked)
  {
    summary.first_position_error = pos;
    summary.first_orientation_error = ori;
    summary.first_checked = true;
  }
  if (pos <= 0.001 && ori <= 0.01)
  {
    return true;
  }
  if (width < 1e-5)
  {
    summary.max_resolution = true;
    return false;
  }

  std::vector<double> seed;
  mid.copyJointGroupPositions(group, seed);
  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes error;
  summary.midpoint_ik_calls += 1;
  const bool ik_ok = solver.searchPositionIK(to_msg(mid_pose), seed, ik_timeout, solution, error);
  if (!ik_ok)
  {
    summary.midpoint_ik_failures += 1;
    if (error.val == moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT)
    {
      summary.ik_timeout = true;
    }
    return false;
  }
  mid.setJointGroupPositions(group, solution);
  mid.updateLinkTransforms();
  const double half = width / 2.0;
  return validate_interval(model, group, start, mid, start_pose, mid_pose, solver, ik_timeout, half, depth + 1, summary) &&
         validate_interval(model, group, mid, end, mid_pose, end_pose, solver, ik_timeout, half, depth + 1, summary);
}

void report_solution(
  const char * label, const moveit::core::RobotModelConstPtr & model, const moveit::core::JointModelGroup * group,
  const std::vector<double> & seed, const std::vector<double> & solution, const Eigen::Isometry3d & target,
  bool ik_success, int error_code, kinematics::KinematicsBase & solver, double ik_timeout)
{
  std::cout << std::setprecision(17);
  std::cout << label << " IK_SUCCESS " << (ik_success ? 1 : 0) << " ERROR " << error_code;
  if (!ik_success)
  {
    if (error_code == moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT)
    {
      std::cout << " PICK_IK_DIAGNOSTIC_RESULT TIME_BUDGET_LIMITED";
    }
    std::cout << "\n";
    return;
  }
  moveit::core::RobotState solved(model);
  solved.setToDefaultValues();
  solved.setJointGroupPositions(group, solution);
  const bool inside = solved.satisfiesBounds(group);
  const Eigen::Isometry3d reached = fk_tool(model, group, solution);
  const Eigen::Isometry3d start_pose = fk_tool(model, group, seed);
  moveit::core::RobotState start(model);
  start.setToDefaultValues();
  start.setJointGroupPositions(group, seed);
  start.update();
  solved.update();
  PrecisionSummary summary;
  const bool passed =
    validate_interval(model, group, start, solved, start_pose, target, solver, ik_timeout, 0.5, 0, summary);
  summary.passed = passed && !summary.max_resolution && summary.midpoint_ik_failures == 0;
  std::cout << " LIMIT_OK " << (inside ? 1 : 0)
            << " JOINT_DELTA_RAD " << max_joint_delta(seed, solution)
            << " END_POSITION_ERROR_M " << position_error(target, reached)
            << " END_ORIENTATION_ERROR_RAD " << orientation_error(target, reached)
            << " FIRST_MID_POSITION_ERROR_M " << summary.first_position_error
            << " FIRST_MID_ORIENTATION_ERROR_RAD " << summary.first_orientation_error
            << " SUBDIVISION_REQUIRED " << ((summary.first_position_error > 0.001 || summary.first_orientation_error > 0.01) ? 1 : 0)
            << " MAX_DEPTH " << summary.max_depth
            << " MIDPOINT_IK_CALLS " << summary.midpoint_ik_calls
            << " MIDPOINT_IK_FAILURES " << summary.midpoint_ik_failures
            << " MAX_RESOLUTION_TERMINATION " << (summary.max_resolution ? 1 : 0)
            << " CARTESIAN_PRECISION_PASS " << (summary.passed ? 1 : 0);
  if (summary.ik_timeout)
  {
    std::cout << " PICK_IK_DIAGNOSTIC_RESULT TIME_BUDGET_LIMITED";
  }
  std::cout << "\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3)
  {
    std::cerr << "usage: pick_ik_benchmark urdf srdf\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pick_ik_benchmark");
  auto urdf = urdf::parseURDF(read_file(argv[1]));
  auto srdf = std::make_shared<srdf::Model>();
  if (!urdf || !srdf->initString(*urdf, read_file(argv[2])))
  {
    std::cerr << "FAIL_ROBOT_MODEL_LOAD\n";
    return 3;
  }
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  auto * group = model->getJointModelGroup("ur_manipulator");
  if (group == nullptr)
  {
    std::cerr << "FAIL_ROBOT_MODEL_LOAD group\n";
    return 3;
  }

  auto kdl = std::make_shared<kdl_kinematics_plugin::KDLKinematicsPlugin>();
  if (!kdl->initialize(node, *model, "ur_manipulator", "base_link", {"tool0"}, 0.005))
  {
    std::cerr << "FAIL_KDL_INITIALIZE\n";
    return 4;
  }
  pluginlib::ClassLoader<kinematics::KinematicsBase> loader("moveit_core", "kinematics::KinematicsBase");
  std::shared_ptr<kinematics::KinematicsBase> pick;
  try
  {
    pick = loader.createSharedInstance("pick_ik/PickIkPlugin");
  }
  catch (const std::exception & error)
  {
    std::cerr << "FAIL_PLUGIN_LOAD " << error.what() << "\n";
    return 4;
  }
  if (!pick->initialize(node, *model, "ur_manipulator", "base_link", {"tool0"}, 0.005))
  {
    std::cerr << "FAIL_PICK_INITIALIZE\n";
    return 4;
  }

  const double kdl_timeout = 0.5;
  const double pick_timeout = 0.005;
  std::cout << "KDL_TIMEOUT_S " << kdl_timeout << "\n";
  std::cout << "PICK_IK_TIMEOUT_S " << pick_timeout << "\n";
  std::cout << "INITIAL_INTERVAL_WIDTH 0.5\n";
  std::cout << "CARTESIAN_PRECISION_M 0.001\n";
  std::cout << "CARTESIAN_PRECISION_RAD 0.01\n";
  std::cout << "MAX_RESOLUTION 1e-5\n";
  std::cout << "TRIAL_COUNT 1\n";
  std::cout << "PICK_IK_PARAMETER_OVERRIDE NONE\n";
  std::cout << "TIMEOUT_RAISED_AFTER_RESULT NO\n";

  Fixture good_a{
    "GOOD_A",
    {3.618, 4.58254, -2.63423, 6.24082, -4.93859, -1.14426},
    {3.63235, 4.56966, -2.63458, 6.27626, -4.99797, -1.15453},
    false,
    Eigen::Isometry3d::Identity()};
  Fixture good_b{
    "GOOD_B",
    {3.56664, 4.65491, -2.5463, 5.83822, -4.77645, -1.14985},
    {3.618, 4.58254, -2.63423, 6.24082, -4.93859, -1.14426},
    false,
    Eigen::Isometry3d::Identity()};
  Fixture pathological{
    "PATHOLOGICAL",
    {3.41001, -1.74927, -2.56452, 6.27274, -4.56621, -1.23097},
    {5.76067, 4.81427, 2.58762, 3.36533, 5.08391, -2.00804},
    true,
    xyzq(0.372859, -0.0191847, 0.164682, 0.976406, -0.0205977, 0.184038, -0.11107)};
  const std::vector<Fixture> fixtures = {good_a, good_b, pathological};

  for (const auto & fixture : fixtures)
  {
    const Eigen::Isometry3d recorded_fk = fk_tool(model, group, fixture.recorded);
    const Eigen::Isometry3d target = fixture.has_logged_target ? fixture.logged_target : recorded_fk;
    std::cout << fixture.name << " TARGET_SOURCE "
              << (fixture.has_logged_target ? "LOGGED_CARTESIAN_END" : "FK_OF_RECORDED_KDL_RESULT") << "\n";
    report_solution(
      (std::string(fixture.name) + " KDL_RECORDED").c_str(), model, group, fixture.seed, fixture.recorded, target, true,
      1, *kdl, kdl_timeout);

    std::vector<double> kdl_solution;
    moveit_msgs::msg::MoveItErrorCodes kdl_error;
    const bool kdl_ok = kdl->searchPositionIK(to_msg(target), fixture.seed, kdl_timeout, kdl_solution, kdl_error);
    report_solution(
      (std::string(fixture.name) + " KDL_REPLAY").c_str(), model, group, fixture.seed,
      kdl_ok ? kdl_solution : fixture.seed, target, kdl_ok, kdl_error.val, *kdl, kdl_timeout);

    std::vector<double> pick_solution;
    moveit_msgs::msg::MoveItErrorCodes pick_error;
    const bool pick_ok = pick->searchPositionIK(to_msg(target), fixture.seed, pick_timeout, pick_solution, pick_error);
    report_solution(
      (std::string(fixture.name) + " PICK_IK").c_str(), model, group, fixture.seed,
      pick_ok ? pick_solution : fixture.seed, target, pick_ok, pick_error.val, *pick, pick_timeout);
  }
  std::cout << "FULL_741_WAYPOINT_ACCEPTANCE_EXECUTED NO\n";
  std::cout << "JUSTIFIABLE_ALTERNATIVE_FOUND NO\n";
  rclcpp::shutdown();
  return 0;
}
