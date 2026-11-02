// P-022-AB 실험 B 계측. MoveIt 2.12.4 cartesian_interpolator.cpp의 구간 검사와 같은 순서다.
// 라이브러리 computeCartesianPath는 호출하지 않는다. group default IK timeout은 덮어쓰지 않는다.
// run 20만 최대 20회. 20/33이 나온 실행만 실패 구간을 남긴다. 궤적 파일은 쓰지 않는다.

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
#include <iomanip>

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

struct Check
{
  int waypoint_index = -1;
  int step_index = -1;
  int depth = -1;
  std::string result;
  double width = 0;
  double position_error = 0;
  double orientation_error = 0;
  double joint_endpoint_max_abs_delta = 0;
  double joint_delta_from_seed = 0;
  bool ik_called = false;
  bool ik_success = false;
  std::vector<double> cartesian_start;
  std::vector<double> cartesian_end;
  std::vector<double> expected_midpoint;
  std::vector<double> fk_midpoint;
  std::vector<double> joint_start;
  std::vector<double> joint_end;
  std::vector<double> joint_midpoint;
  std::vector<double> ik_seed;
  std::vector<double> ik_result;
};

struct SegmentResult
{
  double percentage = 0;
  bool ik_false = false;
};

int g_waypoint = -1;
int g_step = -1;
std::vector<Check> g_checks;

std::vector<double> pose_xyzq(const Eigen::Isometry3d & pose)
{
  const Eigen::Quaterniond rotation(pose.linear());
  return {pose.translation().x(), pose.translation().y(), pose.translation().z(), rotation.x(), rotation.y(),
          rotation.z(), rotation.w()};
}

std::vector<double> joint_values(const moveit::core::RobotState & state, const moveit::core::JointModelGroup * group)
{
  std::vector<double> values;
  state.copyJointGroupPositions(group, values);
  return values;
}

double max_abs_difference(const std::vector<double> & left, const std::vector<double> & right)
{
  double best = 0;
  const std::size_t count = std::min(left.size(), right.size());
  for (std::size_t index = 0; index < count; ++index)
  {
    best = std::max(best, std::abs(left[index] - right[index]));
  }
  return best;
}

bool validate_and_improve(const moveit::core::RobotState & start_state, const moveit::core::RobotState & end_state,
                          const Eigen::Isometry3d & start_pose, const Eigen::Isometry3d & end_pose,
                          std::vector<std::shared_ptr<moveit::core::RobotState>> & traj, double & percentage,
                          double width, const moveit::core::JointModelGroup * group, const moveit::core::LinkModel * link,
                          const moveit::core::CartesianPrecision & precision, const Eigen::Isometry3d & link_offset,
                          int depth)
{
  moveit::core::RobotState mid_state(start_state.getRobotModel());
  start_state.interpolate(end_state, 0.5, mid_state);
  const Eigen::Isometry3d fk_pose = mid_state.getGlobalLinkTransform(link) * link_offset;
  Eigen::Isometry3d mid_pose(Eigen::Quaterniond(start_pose.linear()).slerp(0.5, Eigen::Quaterniond(end_pose.linear())));
  mid_pose.translation() = 0.5 * (start_pose.translation() + end_pose.translation());

  const double linear_distance = (mid_pose.translation() - fk_pose.translation()).norm();
  const double angular_distance =
    Eigen::Quaterniond(mid_pose.linear()).angularDistance(Eigen::Quaterniond(fk_pose.linear()));

  Check check;
  check.waypoint_index = g_waypoint;
  check.step_index = g_step;
  check.depth = depth;
  check.width = width;
  check.position_error = linear_distance;
  check.orientation_error = angular_distance;
  check.cartesian_start = pose_xyzq(start_pose);
  check.cartesian_end = pose_xyzq(end_pose);
  check.expected_midpoint = pose_xyzq(mid_pose);
  check.fk_midpoint = pose_xyzq(fk_pose);
  check.joint_start = joint_values(start_state, group);
  check.joint_end = joint_values(end_state, group);
  check.joint_midpoint = joint_values(mid_state, group);
  check.joint_endpoint_max_abs_delta = max_abs_difference(check.joint_start, check.joint_end);

  if (linear_distance <= precision.translational && angular_distance <= precision.rotational)
  {
    check.result = "PRECISION_MET";
    g_checks.push_back(check);
    traj.push_back(std::make_shared<moveit::core::RobotState>(end_state));
    return true;
  }
  if (width < precision.max_resolution)
  {
    check.result = "MAX_RESOLUTION";
    g_checks.push_back(check);
    return false;
  }

  check.ik_called = true;
  check.ik_seed = check.joint_midpoint;
  const bool ik_ok = mid_state.setFromIK(group, mid_pose * link_offset.inverse(), link->getName(), 0.0);
  check.ik_success = ik_ok;
  if (ik_ok)
  {
    check.ik_result = joint_values(mid_state, group);
    check.joint_delta_from_seed = max_abs_difference(check.ik_seed, check.ik_result);
  }
  if (!ik_ok)
  {
    check.result = "MIDPOINT_IK_FALSE";
    g_checks.push_back(check);
    return false;
  }
  check.result = "SUBDIVIDE";
  g_checks.push_back(check);

  const double half_width = width / 2.0;
  const double old_percentage = percentage;
  percentage = percentage - half_width;
  if (!validate_and_improve(start_state, mid_state, start_pose, mid_pose, traj, percentage, half_width, group, link,
                            precision, link_offset, depth + 1))
  {
    return false;
  }
  percentage = old_percentage;
  return validate_and_improve(mid_state, end_state, mid_pose, end_pose, traj, percentage, half_width, group, link,
                              precision, link_offset, depth + 1);
}

SegmentResult compute_segment(const moveit::core::RobotState * start_state, const moveit::core::JointModelGroup * group,
                              std::vector<std::shared_ptr<moveit::core::RobotState>> & traj,
                              const moveit::core::LinkModel * link, const Eigen::Isometry3d & target, bool global,
                              const moveit::core::MaxEEFStep & max_step, const moveit::core::CartesianPrecision & precision,
                              const Eigen::Isometry3d & link_offset)
{
  moveit::core::RobotState state(*start_state);
  for (const moveit::core::JointModel * joint : group->getContinuousJointModels())
  {
    state.enforceBounds(joint);
  }

  const Eigen::Isometry3d start_pose = state.getGlobalLinkTransform(link) * link_offset;
  const Eigen::Isometry3d inv_offset = link_offset.inverse();
  const Eigen::Isometry3d rotated_target = global ? target : start_pose * target;
  const Eigen::Quaterniond start_quaternion(start_pose.linear());
  const Eigen::Quaterniond target_quaternion(rotated_target.linear());
  const double rotation_distance = start_quaternion.angularDistance(target_quaternion);
  const double translation_distance = (rotated_target.translation() - start_pose.translation()).norm();

  std::size_t translation_steps = 0;
  if (max_step.translation > 0.0)
  {
    translation_steps = static_cast<std::size_t>(std::floor(translation_distance / max_step.translation));
  }
  std::size_t rotation_steps = 0;
  if (max_step.rotation > 0.0)
  {
    rotation_steps = static_cast<std::size_t>(std::floor(rotation_distance / max_step.rotation));
  }
  const std::size_t steps = std::max(translation_steps, rotation_steps) + 1;

  traj.clear();
  traj.push_back(std::make_shared<moveit::core::RobotState>(*start_state));

  double last_valid_percentage = 0.0;
  Eigen::Isometry3d prev_pose = start_pose;
  moveit::core::RobotState prev_state(state);
  for (std::size_t index = 1; index <= steps; ++index)
  {
    g_step = static_cast<int>(index);
    const double percentage = static_cast<double>(index) / static_cast<double>(steps);
    double running = percentage;
    Eigen::Isometry3d pose(start_quaternion.slerp(percentage, target_quaternion));
    pose.translation() = percentage * rotated_target.translation() + (1.0 - percentage) * start_pose.translation();

    const std::vector<double> seed = joint_values(state, group);
    const bool ik_ok = state.setFromIK(group, pose * inv_offset, link->getName(), 0.0);
    Check endpoint;
    endpoint.waypoint_index = g_waypoint;
    endpoint.step_index = g_step;
    endpoint.depth = -1;
    endpoint.width = 1.0 / static_cast<double>(steps);
    endpoint.result = ik_ok ? "ENDPOINT_IK_TRUE" : "ENDPOINT_IK_FALSE";
    endpoint.ik_called = true;
    endpoint.ik_success = ik_ok;
    endpoint.ik_seed = seed;
    endpoint.cartesian_end = pose_xyzq(pose);
    if (ik_ok)
    {
      endpoint.ik_result = joint_values(state, group);
      endpoint.joint_delta_from_seed = max_abs_difference(seed, endpoint.ik_result);
    }
    g_checks.push_back(endpoint);
    if (!ik_ok || !validate_and_improve(prev_state, state, prev_pose, pose, traj, running, endpoint.width, group, link,
                                        precision, link_offset, 0))
    {
      SegmentResult failed;
      failed.percentage = last_valid_percentage;
      failed.ik_false = !ik_ok;
      return failed;
    }
    prev_pose = pose;
    prev_state = state;
    last_valid_percentage = percentage;
  }
  return SegmentResult{last_valid_percentage, false};
}

double compute_waypoints(const moveit::core::RobotState * start_state, const moveit::core::JointModelGroup * group,
                         std::vector<std::shared_ptr<moveit::core::RobotState>> & traj,
                         const moveit::core::LinkModel * link, const EigenSTL::vector_Isometry3d & waypoints,
                         const moveit::core::MaxEEFStep & max_step, const moveit::core::CartesianPrecision & precision,
                         int * completed, int * failing, bool * ik_false)
{
  double percentage_solved = 0.0;
  *completed = 0;
  *failing = -1;
  *ik_false = false;
  const moveit::core::RobotState * current = start_state;
  for (std::size_t index = 0; index < waypoints.size(); ++index)
  {
    g_waypoint = static_cast<int>(index);
    std::vector<std::shared_ptr<moveit::core::RobotState>> waypoint_traj;
    const SegmentResult segment = compute_segment(current, group, waypoint_traj, link, waypoints[index], true, max_step,
                                                  precision, Eigen::Isometry3d::Identity());
    auto begin = waypoint_traj.begin();
    if (index > 0 && !waypoint_traj.empty())
    {
      std::advance(begin, 1);
    }
    traj.insert(traj.end(), begin, waypoint_traj.end());
    if (std::fabs(segment.percentage - 1.0) < std::numeric_limits<double>::epsilon())
    {
      percentage_solved = static_cast<double>(index + 1) / static_cast<double>(waypoints.size());
      *completed = static_cast<int>(index + 1);
    }
    else
    {
      percentage_solved += segment.percentage / static_cast<double>(waypoints.size());
      *failing = static_cast<int>(index);
      *ik_false = segment.ik_false;
      break;
    }
    current = traj.back().get();
  }
  return percentage_solved;
}

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
}  // namespace

double limit_margin(const moveit::core::RobotModelConstPtr & model, const std::vector<std::string> & names,
                     const std::vector<double> & values, int * which)
{
  double best = std::numeric_limits<double>::infinity();
  *which = -1;
  for (std::size_t index = 0; index < values.size() && index < names.size(); ++index)
  {
    const auto * joint = model->getJointModel(names[index]);
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

double min_singular(const moveit::core::RobotModelConstPtr & model, const moveit::core::JointModelGroup * group,
                    const std::vector<std::string> & names, const std::vector<double> & values, bool * valid)
{
  *valid = false;
  if (values.empty())
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  for (std::size_t index = 0; index < values.size() && index < names.size(); ++index)
  {
    state.setVariablePosition(names[index], values[index]);
  }
  state.update();
  const Eigen::MatrixXd jacobian = state.getJacobian(group);
  if (jacobian.size() == 0)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  const auto singular = svd.singularValues();
  if (singular.size() == 0)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  *valid = true;
  return singular.minCoeff();
}

int main(int argc, char ** argv)
{
  if (argc != 5)
  {
    std::cerr << "usage: p022ab_trace urdf srdf runs output\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("p022ab_trace");
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
  auto solver = std::make_shared<kdl_kinematics_plugin::KDLKinematicsPlugin>();
  if (group == nullptr || tool == nullptr ||
      !solver->initialize(node, *model, "ur_manipulator", "base_link", {"tool0"}, 0.005))
  {
    std::cerr << "FAIL_ROBOT_MODEL_LOAD\n";
    return 3;
  }
  model->setKinematicsAllocators(
    {{"ur_manipulator", [solver](const moveit::core::JointModelGroup *) { return solver; }}});
  std::cout << "DEFAULT_GROUP_IK_TIMEOUT_S " << group->getDefaultIKTimeout() << "\n";

  const moveit::core::CartesianPrecision precision;
  const moveit::core::MaxEEFStep step(0.005, 0.0);
  const int max_trials = 20;
  const double start_timeout = 0.005;
  const auto runs = read_runs(argv[3]);
  if (runs.size() < 21)
  {
    throw std::runtime_error("run 20 missing");
  }
  const auto & poses = runs[20];
  const auto joint_names = group->getActiveJointModelNames();
  std::vector<std::string> statuses(max_trials);
  std::vector<double> fractions(max_trials, std::numeric_limits<double>::quiet_NaN());
  std::vector<Check> captured;
  int captured_trial = -1;
  int captured_completed = -1;
  int captured_failing = -1;
  bool captured_ik_false = false;
  int match_count = 0;

  for (int trial = 0; trial < max_trials; ++trial)
  {
    g_checks.clear();
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    state.update();
    if (!state.setFromIK(group, poses.front().pose, std::string("tool0"), start_timeout))
    {
      statuses[trial] = "START_IK_FAILED";
      std::cout << "TRIAL " << trial << " START_IK_FAILED\n" << std::flush;
      continue;
    }
    EigenSTL::vector_Isometry3d waypoints;
    for (std::size_t index = 1; index < poses.size(); ++index)
    {
      waypoints.push_back(poses[index].pose);
    }
    std::vector<std::shared_ptr<moveit::core::RobotState>> trajectory;
    int completed = 0;
    int failing = -1;
    bool path_ik_false = false;
    const double fraction =
      compute_waypoints(&state, group, trajectory, tool, waypoints, step, precision, &completed, &failing,
                        &path_ik_false);
    fractions[trial] = fraction;
    statuses[trial] = "OK";
    const bool matched = poses.size() == 34 && std::abs(fraction * 33.0 - 20.0) < 1e-4;
    std::cout << "TRIAL " << trial << " FRACTION " << std::setprecision(17) << fraction
              << (matched ? " MATCH_20_33\n" : "\n") << std::flush;
    if (matched)
    {
      ++match_count;
      if (captured_trial < 0)
      {
        captured = g_checks;
        captured_trial = trial;
        captured_completed = completed;
        captured_failing = failing;
        captured_ik_false = path_ik_false;
      }
    }
    g_checks.clear();
  }

  std::ofstream output(argv[4]);
  output << std::setprecision(17);
  output << "{\n  \"gate\": \"P-022-AB\",\n  \"experiment\": \"B_INSTRUMENTED\",\n"
         << "  \"instrumentation\": \"SOURCE_ORDER_COPY\",\n"
         << "  \"library_compute_cartesian_path_called\": false,\n"
         << "  \"source_ik_function\": \"RobotState::setFromIK\",\n"
         << "  \"source_interval_function\": \"CartesianInterpolator::validateAndImproveInterval\",\n"
         << "  \"source_version\": \"MoveIt 2.12.4 cartesian_interpolator.cpp\",\n"
         << "  \"set_default_ik_timeout\": false,\n  \"start_ik_timeout_s\": 0.005,\n"
         << "  \"default_group_ik_timeout_s\": " << group->getDefaultIKTimeout()
         << ",\n  \"max_trials\": " << max_trials
         << ",\n  \"run_id\": 20,\n  \"pose_count\": " << poses.size()
         << ",\n  \"match_20_over_33_count\": " << match_count
         << ",\n  \"captured_trial\": " << captured_trial
         << ",\n  \"completed_interval_count\": " << captured_completed
         << ",\n  \"failing_interval_index\": " << captured_failing
         << ",\n  \"path_ik_false\": " << (captured_ik_false ? "true" : "false")
         << ",\n  \"cause_status\": \""
         << (match_count > 0 ? "INSTRUMENTED_20_OVER_33_REPRODUCED" : "INSTRUMENTED_20_OVER_33_NOT_REPRODUCED")
         << "\",\n  \"trials\": [\n";
  for (int trial = 0; trial < max_trials; ++trial)
  {
    if (trial)
    {
      output << ",\n";
    }
    output << "    {\"trial\": " << trial << ", \"api_status\": \"" << statuses[trial] << "\", \"cartesian_fraction\": ";
    write_number(output, fractions[trial]);
    output << ", \"matches_20_over_33\": "
           << (poses.size() == 34 && std::isfinite(fractions[trial]) && std::abs(fractions[trial] * 33.0 - 20.0) < 1e-4 ?
                 "true" :
                 "false")
           << "}";
  }
  output << "\n  ],\n  \"failing_interval_checks\": [\n";
  bool first = true;
  for (const auto & check : captured)
  {
    if (check.waypoint_index != captured_failing)
    {
      continue;
    }
    if (!first)
    {
      output << ",\n";
    }
    first = false;
    const std::vector<double> & observed = check.joint_end.empty() ? check.joint_midpoint : check.joint_end;
    int which = -1;
    const double margin = limit_margin(model, joint_names, observed, &which);
    bool jacobian_valid = false;
    const double singular = min_singular(model, group, joint_names, observed, &jacobian_valid);
    output << "    {\"interval_index\": " << check.waypoint_index << ", \"step_index\": " << check.step_index
           << ", \"refinement_depth\": " << check.depth << ", \"validation_result\": \"" << check.result
           << "\", \"interval_width\": ";
    write_number(output, check.width);
    output << ", \"position_error_m\": ";
    write_number(output, check.position_error);
    output << ", \"orientation_error_rad\": ";
    write_number(output, check.orientation_error);
    output << ", \"joint_endpoint_max_abs_delta_rad\": ";
    write_number(output, check.joint_endpoint_max_abs_delta);
    output << ", \"ik_called\": " << (check.ik_called ? "true" : "false")
           << ", \"ik_success\": " << (check.ik_success ? "true" : "false")
           << ", \"joint_delta_from_seed_rad\": ";
    write_number(output, check.ik_called && check.ik_success ? check.joint_delta_from_seed :
                                                                  std::numeric_limits<double>::quiet_NaN());
    output << ", \"limit_distance_rad\": ";
    write_number(output, margin);
    output << ", \"limit_joint_index\": " << which << ", \"min_singular_value\": ";
    write_number(output, singular);
    output << ", \"jacobian_valid\": " << (jacobian_valid ? "true" : "false");
    output << ", \"cartesian_start\": ";
    write_vector(output, check.cartesian_start);
    output << ", \"cartesian_end\": ";
    write_vector(output, check.cartesian_end);
    output << ", \"expected_cartesian_midpoint\": ";
    write_vector(output, check.expected_midpoint);
    output << ", \"fk_joint_midpoint\": ";
    write_vector(output, check.fk_midpoint);
    output << ", \"joint_start\": ";
    write_vector(output, check.joint_start);
    output << ", \"joint_end\": ";
    write_vector(output, check.joint_end);
    output << ", \"joint_midpoint\": ";
    write_vector(output, check.joint_midpoint);
    output << ", \"ik_seed\": ";
    write_vector(output, check.ik_seed);
    output << ", \"ik_result\": ";
    if (check.ik_result.empty())
    {
      output << "null";
    }
    else
    {
      write_vector(output, check.ik_result);
    }
    output << "}";
  }
  output << "\n  ]\n}\n";
  std::cout << "INSTRUMENTED_MATCH_COUNT " << match_count << "\n";
  rclcpp::shutdown();
  return 0;
}
