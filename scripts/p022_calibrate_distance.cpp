// 독립 calibration. callback은 넣지 않는다. 20회를 끝까지 돈 뒤에 M을 한 번 계산한다.
// partial fraction과 시작 IK 실패는 M에 넣지 않는다.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
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

class DeltaKDL : public kdl_kinematics_plugin::KDLKinematicsPlugin
{
public:
  mutable std::string phase;
  mutable double run_max = 0;
  mutable int run_joint = -1;
  mutable int run_count = 0;

  bool searchPositionIK(const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
                        double timeout, const std::vector<double> & consistency_limits, std::vector<double> & solution,
                        moveit_msgs::msg::MoveItErrorCodes & error_code,
                        const kinematics::KinematicsQueryOptions & options) const override
  {
    const bool ok = kdl_kinematics_plugin::KDLKinematicsPlugin::searchPositionIK(
      ik_pose, ik_seed_state, timeout, consistency_limits, solution, error_code, options);
    if (ok && phase == "PATH")
    {
      double best = 0;
      int joint = -1;
      for (std::size_t index = 0; index < solution.size() && index < ik_seed_state.size(); ++index)
      {
        const double delta = std::abs(solution[index] - ik_seed_state[index]);
        if (delta > best)
        {
          best = delta;
          joint = static_cast<int>(index);
        }
      }
      ++run_count;
      if (best > run_max)
      {
        run_max = best;
        run_joint = joint;
      }
    }
    return ok;
  }
};

std::string read_file(const std::string & path)
{
  std::ifstream input(path);
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

std::vector<std::vector<Pose>> read_runs(const std::string & path)
{
  std::ifstream input(path);
  std::vector<std::vector<Pose>> runs;
  std::string tag;
  while (input >> tag)
  {
    int run_id = 0;
    int count = 0;
    input >> run_id >> count;
    std::vector<Pose> poses;
    for (int index = 0; index < count; ++index)
    {
      double x, y, z, qx, qy, qz, qw;
      input >> x >> y >> z >> qx >> qy >> qz >> qw;
      Pose pose;
      pose.pose = Eigen::Isometry3d::Identity();
      pose.pose.translation() = Eigen::Vector3d(x, y, z);
      pose.pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
      poses.push_back(pose);
    }
    if (run_id != static_cast<int>(runs.size()))
    {
      throw std::runtime_error("run id");
    }
    runs.push_back(poses);
  }
  return runs;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 5)
  {
    std::cerr << "usage: p022_calibrate_distance urdf srdf runs output\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("p022_calibrate_distance");
  auto urdf = urdf::parseURDF(read_file(argv[1]));
  auto srdf = std::make_shared<srdf::Model>();
  if (!urdf || !srdf->initString(*urdf, read_file(argv[2])))
  {
    std::cerr << "FAIL_ROBOT_MODEL_LOAD\n";
    return 3;
  }
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  auto * group = model->getJointModelGroup("ur_manipulator");
  const auto * tool = model->getLinkModel("tool0");
  auto solver = std::make_shared<DeltaKDL>();
  if (group == nullptr || tool == nullptr ||
      !solver->initialize(node, *model, "ur_manipulator", "base_link", {"tool0"}, 0.005))
  {
    std::cerr << "FAIL_ROBOT_MODEL_LOAD\n";
    return 3;
  }
  model->setKinematicsAllocators(
    {{"ur_manipulator", [solver](const moveit::core::JointModelGroup *) { return solver; }}});

  const int trial_limit = 20;
  const double start_timeout = 0.005;
  const auto runs = read_runs(argv[3]);
  const moveit::core::MaxEEFStep step(0.005, 0.0);
  const moveit::core::CartesianPrecision precision;
  int start_failed = 0;
  int fraction_one = 0;
  int partial = 0;
  int fraction_zero = 0;
  int transitions = 0;
  double global_max = 0;
  int global_joint = -1;
  int global_trial = -1;
  int global_run = -1;

  for (int trial = 0; trial < trial_limit; ++trial)
  {
    std::cout << "TRIAL " << trial << "\n" << std::flush;
    for (std::size_t run_id = 0; run_id < runs.size(); ++run_id)
    {
      solver->phase = "START";
      solver->run_max = 0;
      solver->run_joint = -1;
      solver->run_count = 0;
      moveit::core::RobotState state(model);
      state.setToDefaultValues();
      state.update();
      if (!state.setFromIK(group, runs[run_id].front().pose, std::string("tool0"), start_timeout))
      {
        ++start_failed;
        continue;
      }
      EigenSTL::vector_Isometry3d waypoints;
      for (std::size_t index = 1; index < runs[run_id].size(); ++index)
      {
        waypoints.push_back(runs[run_id][index].pose);
      }
      std::vector<std::shared_ptr<moveit::core::RobotState>> trajectory;
      solver->phase = "PATH";
      auto percentage = moveit::core::CartesianInterpolator::computeCartesianPath(
        &state, group, trajectory, tool, waypoints, true, step, precision);
      const double fraction = static_cast<double>(percentage);
      if (std::abs(fraction - 1.0) < 1e-9)
      {
        ++fraction_one;
        transitions += solver->run_count;
        if (solver->run_max > global_max)
        {
          global_max = solver->run_max;
          global_joint = solver->run_joint;
          global_trial = trial;
          global_run = static_cast<int>(run_id);
        }
      }
      else if (fraction == 0.0)
      {
        ++fraction_zero;
      }
      else
      {
        ++partial;
      }
    }
  }

  const char * result = "INSUFFICIENT_CALIBRATION_COVERAGE";
  const char * callback_note = "NOT_RECONSIDERED";
  if (fraction_one == 0 || global_max <= 0.402594)
  {
    result = "INSUFFICIENT_CALIBRATION_COVERAGE";
  }
  else if (global_max >= 1.85667)
  {
    result = "SEPARATION_NOT_SUPPORTED";
    callback_note = "RECONSIDER";
  }
  else
  {
    result = "SEPARATION_OBSERVED";
  }

  std::ofstream output(argv[4]);
  output << "{\n  \"result\": \"" << result << "\",\n  \"trial_limit\": " << trial_limit
         << ",\n  \"early_stop\": false,\n  \"callback_applied\": false,\n  \"set_default_ik_timeout\": false,\n"
         << "  \"start_ik_failed_count\": " << start_failed << ",\n  \"fraction_one_count\": " << fraction_one
         << ",\n  \"partial_count\": " << partial << ",\n  \"fraction_zero_count\": " << fraction_zero
         << ",\n  \"path_transition_count\": " << transitions << ",\n  \"M\": " << global_max
         << ",\n  \"M_joint_index\": " << global_joint << ",\n  \"M_trial\": " << global_trial
         << ",\n  \"M_run_id\": " << global_run << ",\n  \"T\": null,\n  \"scalar_distance_callback\": \""
         << callback_note << "\"\n}\n";
  std::cout << "RESULT " << result << " M " << global_max << "\n";
  rclcpp::shutdown();
  return 0;
}
