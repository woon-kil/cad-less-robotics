// run 20 joint trace. 기준선 설정을 바꾸지 않는다.
// group default IK timeout은 덮어쓰지 않는다. 궤적 파일은 쓰지 않는다.

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/SVD>

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

struct Call
{
  int repeat = 0;
  int call_index = 0;
  std::string phase;
  double timeout = 0;
  bool success = false;
  int error_code = 0;
  std::vector<double> seed;
  std::vector<double> solution;
  double max_abs_delta = 0;
  double l2_delta = 0;
  double seed_limit_margin = 0;
  int seed_limit_joint = -1;
  double solution_limit_margin = 0;
  int solution_limit_joint = -1;
  double min_singular = 0;
  double condition = 0;
  bool jacobian_valid = false;
  std::vector<double> singular_values;
  double px = 0;
  double py = 0;
  double pz = 0;
  double qx = 0;
  double qy = 0;
  double qz = 0;
  double qw = 0;
};

class TracingKDL : public kdl_kinematics_plugin::KDLKinematicsPlugin
{
public:
  mutable int repeat_id = 0;
  mutable int call_index = 0;
  mutable std::string phase;
  mutable std::vector<Call> calls;

  bool searchPositionIK(const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
                        double timeout, const std::vector<double> & consistency_limits, std::vector<double> & solution,
                        moveit_msgs::msg::MoveItErrorCodes & error_code,
                        const kinematics::KinematicsQueryOptions & options) const override
  {
    Call call;
    call.repeat = repeat_id;
    call.call_index = call_index;
    call.phase = phase;
    call.timeout = timeout;
    call.seed = ik_seed_state;
    call.px = ik_pose.position.x;
    call.py = ik_pose.position.y;
    call.pz = ik_pose.position.z;
    call.qx = ik_pose.orientation.x;
    call.qy = ik_pose.orientation.y;
    call.qz = ik_pose.orientation.z;
    call.qw = ik_pose.orientation.w;
    int joint_index = -1;
    call.seed_limit_margin = limit_margin(ik_seed_state, &joint_index);
    call.seed_limit_joint = joint_index;

    const bool ok = kdl_kinematics_plugin::KDLKinematicsPlugin::searchPositionIK(
      ik_pose, ik_seed_state, timeout, consistency_limits, solution, error_code, options);
    call.success = ok;
    call.error_code = error_code.val;
    if (ok)
    {
      call.solution = solution;
      double sum = 0;
      call.max_abs_delta = 0;
      for (std::size_t index = 0; index < solution.size() && index < ik_seed_state.size(); ++index)
      {
        const double delta = solution[index] - ik_seed_state[index];
        sum += delta * delta;
        call.max_abs_delta = std::max(call.max_abs_delta, std::abs(delta));
      }
      call.l2_delta = std::sqrt(sum);
      call.solution_limit_margin = limit_margin(solution, &joint_index);
      call.solution_limit_joint = joint_index;
      fill_jacobian(solution, call);
    }
    calls.push_back(call);
    ++call_index;
    return ok;
  }

private:
  double limit_margin(const std::vector<double> & values, int * which) const
  {
    double best = std::numeric_limits<double>::infinity();
    *which = -1;
    const auto names = getJointNames();
    for (std::size_t index = 0; index < values.size() && index < names.size(); ++index)
    {
      const auto * joint = robot_model_->getJointModel(names[index]);
      const auto & bounds = joint->getVariableBounds().front();
      const double margin = std::min(values[index] - bounds.min_position_, bounds.max_position_ - values[index]);
      if (margin < best)
      {
        best = margin;
        *which = static_cast<int>(index);
      }
    }
    return best;
  }

  void fill_jacobian(const std::vector<double> & solution, Call & call) const
  {
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    const auto names = getJointNames();
    for (std::size_t index = 0; index < solution.size() && index < names.size(); ++index)
    {
      state.setVariablePosition(names[index], solution[index]);
    }
    state.update();
    const auto * group = robot_model_->getJointModelGroup("ur_manipulator");
    const Eigen::MatrixXd jacobian = state.getJacobian(group);
    if (jacobian.size() == 0)
    {
      return;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
    const auto values = svd.singularValues();
    if (values.size() == 0)
    {
      return;
    }
    call.min_singular = values.minCoeff();
    call.singular_values.assign(values.data(), values.data() + values.size());
    const double max_singular = values.maxCoeff();
    call.condition = call.min_singular > 0 ? max_singular / call.min_singular : std::numeric_limits<double>::infinity();
    call.jacobian_valid = true;
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

void write_number(std::ostream & output, double value)
{
  if (!std::isfinite(value))
  {
    output << "null";
    return;
  }
  output << value;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 5)
  {
    std::cerr << "usage: p022_capture_partial urdf srdf runs output\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("p022_capture_partial");
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
  auto solver = std::make_shared<TracingKDL>();
  if (group == nullptr || tool == nullptr ||
      !solver->initialize(node, *model, "ur_manipulator", "base_link", {"tool0"}, 0.005))
  {
    std::cerr << "FAIL_ROBOT_MODEL_LOAD\n";
    return 3;
  }
  model->setKinematicsAllocators(
    {{"ur_manipulator", [solver](const moveit::core::JointModelGroup *) { return solver; }}});
  std::cout << "DEFAULT_GROUP_IK_TIMEOUT_S " << group->getDefaultIKTimeout() << "\n";

  const int max_trials = 20;
  const double start_timeout = 0.005;
  const auto runs = read_runs(argv[3]);
  int executed = 0;
  int start_failed = 0;
  int full_fraction = 0;
  int fraction_zero = 0;
  int captured_trial = -1;
  int captured_run = -1;
  double captured_fraction = std::numeric_limits<double>::quiet_NaN();
  std::string result = "PARTIAL_FAILURE_NOT_REPRODUCED_WITHIN_FIXED_TRIALS";

  for (int trial = 0; trial < max_trials; ++trial)
  {
    std::cout << "TRIAL " << trial << "\n" << std::flush;
    bool stop = false;
    for (std::size_t run_id = 0; run_id < runs.size(); ++run_id)
    {
      const auto & poses = runs[run_id];
      solver->calls.clear();
      solver->call_index = 0;
      solver->repeat_id = trial;
      solver->phase = "START";
      moveit::core::RobotState state(model);
      state.setToDefaultValues();
      state.update();
      ++executed;
      if (!state.setFromIK(group, poses.front().pose, std::string("tool0"), start_timeout))
      {
        ++start_failed;
        solver->calls.clear();
        continue;
      }
      EigenSTL::vector_Isometry3d waypoints;
      for (std::size_t index = 1; index < poses.size(); ++index)
      {
        waypoints.push_back(poses[index].pose);
      }
      std::vector<std::shared_ptr<moveit::core::RobotState>> trajectory;
      solver->phase = "PATH";
      const moveit::core::MaxEEFStep step(0.005, 0.0);
      const moveit::core::CartesianPrecision precision;
      auto percentage = moveit::core::CartesianInterpolator::computeCartesianPath(
        &state, group, trajectory, tool, waypoints, true, step, precision);
      const double fraction = static_cast<double>(percentage);
      trajectory.clear();
      if (fraction > 0.0 && fraction < 1.0 - 1e-9)
      {
        captured_trial = trial;
        captured_run = static_cast<int>(run_id);
        captured_fraction = fraction;
        result = "PASS_PARTIAL_FAILURE_CAPTURED";
        stop = true;
        break;
      }
      if (fraction == 0.0)
      {
        ++fraction_zero;
      }
      else
      {
        ++full_fraction;
      }
      solver->calls.clear();
    }
    if (stop)
    {
      break;
    }
  }

  std::ofstream output(argv[4]);
  output << "{\n  \"result\": \"" << result << "\",\n  \"max_trials\": " << max_trials
         << ",\n  \"set_default_ik_timeout\": false,\n  \"start_ik_timeout_s\": 0.005,\n"
         << "  \"default_group_ik_timeout_s\": " << group->getDefaultIKTimeout()
         << ",\n  \"runs_executed\": " << executed
         << ",\n  \"start_ik_failed_count\": " << start_failed
         << ",\n  \"full_fraction_count\": " << full_fraction
         << ",\n  \"fraction_zero_count\": " << fraction_zero
         << ",\n  \"captured_trial\": " << captured_trial
         << ",\n  \"captured_run_id\": " << captured_run
         << ",\n  \"returned_fraction\": ";
  write_number(output, captured_fraction);
  output << ",\n  \"calls\": [\n";
  bool first = true;
  for (const auto & call : solver->calls)
  {
    if (!first)
    {
      output << ",\n";
    }
    first = false;
    output << "    {\"trial\": " << call.repeat << ", \"call_index\": " << call.call_index
           << ", \"phase\": \"" << call.phase << "\", \"timeout_s\": ";
    write_number(output, call.timeout);
    output << ", \"ik_success\": " << (call.success ? "true" : "false") << ", \"error_code\": " << call.error_code
           << ", \"target_position_m\": [";
    write_number(output, call.px);
    output << ", ";
    write_number(output, call.py);
    output << ", ";
    write_number(output, call.pz);
    output << "], \"seed_joint_state\": [";
    for (std::size_t index = 0; index < call.seed.size(); ++index)
    {
      if (index) output << ", ";
      write_number(output, call.seed[index]);
    }
    output << "], \"result_joint_state\": ";
    if (!call.success)
    {
      output << "null";
    }
    else
    {
      output << "[";
      for (std::size_t index = 0; index < call.solution.size(); ++index)
      {
        if (index) output << ", ";
        write_number(output, call.solution[index]);
      }
      output << "]";
    }
    output << ", \"max_abs_joint_delta_from_seed\": ";
    write_number(output, call.success ? call.max_abs_delta : std::numeric_limits<double>::quiet_NaN());
    output << ", \"l2_joint_delta_from_seed\": ";
    write_number(output, call.success ? call.l2_delta : std::numeric_limits<double>::quiet_NaN());
    output << ", \"seed_distance_to_joint_limit_rad\": ";
    write_number(output, call.seed_limit_margin);
    output << ", \"seed_limit_joint\": " << call.seed_limit_joint
           << ", \"solution_distance_to_joint_limit_rad\": ";
    write_number(output, call.success ? call.solution_limit_margin : std::numeric_limits<double>::quiet_NaN());
    output << ", \"jacobian_singular_values\": [";
    for (std::size_t index = 0; index < call.singular_values.size(); ++index)
    {
      if (index) output << ", ";
      write_number(output, call.singular_values[index]);
    }
    output << "]}";
  }
  output << "\n  ]\n}\n";
  std::cout << "RESULT " << result << " executed " << executed << "\n";
  rclcpp::shutdown();
  return 0;
}
