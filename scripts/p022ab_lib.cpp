// P-022-AB 라이브러리 경로 계측. computeCartesianPath는 라이브러리 호출이다.
// 순서 복사본을 쓰지 않는다. group default IK timeout은 덮어쓰지 않는다.
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

#include <geometry_msgs/msg/pose.hpp>

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

struct Call
{
  int trial = -1;
  std::string phase;
  double timeout_s = 0;
  double elapsed_s = 0;
  bool success = false;
  int error_code = 0;
  bool limits_empty = true;
  bool detail = false;
  std::vector<double> position;
  std::vector<double> quaternion_xyzw;
  std::vector<double> seed;
  std::vector<double> solution;
};

class TracingKDL : public kdl_kinematics_plugin::KDLKinematicsPlugin
{
public:
  mutable bool record_detail = false;
  mutable int trial = -1;
  mutable std::string phase;
  mutable int start_calls = 0;
  mutable int path_calls = 0;
  mutable int unexpected_calls = 0;
  mutable int nonempty_limits_calls = 0;
  mutable std::vector<Call> calls;

  void reset_counts() const
  {
    start_calls = 0;
    path_calls = 0;
    unexpected_calls = 0;
    nonempty_limits_calls = 0;
    calls.clear();
  }

  // 설치본 KDL은 callback 없는 overload를 이 시그니처로 넘긴다. 빈 callback은 기존 경로다.
  bool searchPositionIK(const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
                        double timeout, const std::vector<double> & consistency_limits, std::vector<double> & solution,
                        const IKCallbackFn & solution_callback, moveit_msgs::msg::MoveItErrorCodes & error_code,
                        const kinematics::KinematicsQueryOptions & options) const override
  {
    if (solution_callback)
    {
      ++unexpected_calls;
    }
    const auto started = std::chrono::steady_clock::now();
    const bool ok = kdl_kinematics_plugin::KDLKinematicsPlugin::searchPositionIK(
      ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code, options);
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    note(ik_pose, ik_seed_state, timeout, consistency_limits.empty(), ok, error_code.val, elapsed, solution);
    return ok;
  }

private:
  void note(const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & seed, double timeout, bool limits_empty,
            bool ok, int error_code, double elapsed, const std::vector<double> & solution) const
  {
    if (phase == "PATH")
    {
      ++path_calls;
    }
    else
    {
      ++start_calls;
    }
    if (!limits_empty)
    {
      ++nonempty_limits_calls;
    }
    Call call;
    call.trial = trial;
    call.phase = phase;
    call.timeout_s = timeout;
    call.elapsed_s = elapsed;
    call.success = ok;
    call.error_code = error_code;
    call.limits_empty = limits_empty;
    call.detail = record_detail;
    if (record_detail)
    {
      call.position = {ik_pose.position.x, ik_pose.position.y, ik_pose.position.z};
      call.quaternion_xyzw = {ik_pose.orientation.x, ik_pose.orientation.y, ik_pose.orientation.z, ik_pose.orientation.w};
      call.seed = seed;
      if (ok)
      {
        call.solution = solution;
      }
    }
    calls.push_back(std::move(call));
  }
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

bool matches_20_33(double fraction, int pose_count)
{
  return pose_count == 34 && std::isfinite(fraction) && std::abs(fraction * 33.0 - 20.0) < 1e-4;
}

struct Trial
{
  int index = 0;
  bool record_detail = false;
  bool start_ik_success = false;
  double start_elapsed_s = 0;
  double path_elapsed_s = std::numeric_limits<double>::quiet_NaN();
  double fraction = std::numeric_limits<double>::quiet_NaN();
  int start_calls = 0;
  int path_calls = 0;
  bool path_miss = false;
  bool exception = false;
  std::string exception_text;
  bool last_path_success = false;
  bool last_path_known = false;
  double last_path_timeout_s = std::numeric_limits<double>::quiet_NaN();
  std::string stop_class;
};

std::string classify_stop(const Trial & trial)
{
  if (trial.exception)
  {
    return "UNCONFIRMED";
  }
  if (!trial.start_ik_success)
  {
    return "START_IK_FAILED";
  }
  if (trial.path_miss || trial.path_calls == 0)
  {
    return "UNCONFIRMED";
  }
  if (!std::isfinite(trial.fraction) || trial.fraction >= 1.0 - 1e-12)
  {
    return "NOT_APPLICABLE";
  }
  if (!trial.last_path_known)
  {
    return "UNCONFIRMED";
  }
  if (!trial.last_path_success)
  {
    return "ESTIMATED_INTERVAL_IK_FAILURE";
  }
  return "UNCONFIRMED";
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 5)
  {
    std::cerr << "usage: p022ab_lib urdf srdf runs output\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("p022ab_lib");
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
  const double search_discretization = 0.005;
  if (group == nullptr || tool == nullptr ||
      !solver->initialize(node, *model, "ur_manipulator", "base_link", {"tool0"}, search_discretization))
  {
    std::cerr << "FAIL_ROBOT_MODEL_LOAD\n";
    return 3;
  }
  model->setKinematicsAllocators(
    {{"ur_manipulator", [solver](const moveit::core::JointModelGroup *) { return solver; }}});

  const double start_timeout = 0.005;
  const double group_timeout_before = group->getDefaultIKTimeout();
  const auto runs = read_runs(argv[3]);
  if (runs.size() < 21 || runs[20].poses.size() < 2)
  {
    std::cerr << "FAIL_RUN20\n";
    return 3;
  }
  const auto & poses = runs[20].poses;

  auto run_once = [&](int trial, bool detail, Trial & out) {
    solver->trial = trial;
    solver->record_detail = detail;
    solver->phase = "START";
    const int start_before = solver->start_calls;
    const int path_before = solver->path_calls;
    const int unexpected_before = solver->unexpected_calls;
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    state.update();
    const auto start_began = std::chrono::steady_clock::now();
    out.start_ik_success = state.setFromIK(group, poses.front().pose, std::string("tool0"), start_timeout);
    out.start_elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_began).count();
    out.start_calls = solver->start_calls - start_before;
    if (!out.start_ik_success)
    {
      out.path_calls = 0;
      out.stop_class = classify_stop(out);
      return;
    }
    EigenSTL::vector_Isometry3d waypoints;
    for (std::size_t index = 1; index < poses.size(); ++index)
    {
      waypoints.push_back(poses[index].pose);
    }
    std::vector<std::shared_ptr<moveit::core::RobotState>> trajectory;
    solver->phase = "PATH";
    const auto path_began = std::chrono::steady_clock::now();
    try
    {
      const moveit::core::MaxEEFStep step(0.005, 0.0);
      const moveit::core::CartesianPrecision precision;
      moveit::core::CartesianInterpolator::Percentage fraction =
        moveit::core::CartesianInterpolator::computeCartesianPath(
          &state, group, trajectory, tool, waypoints, true, step, precision);
      out.fraction = fraction.value;
    }
    catch (const std::exception & error)
    {
      out.exception = true;
      out.exception_text = error.what();
    }
    out.path_elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - path_began).count();
    out.path_calls = solver->path_calls - path_before;
    out.path_miss = out.path_calls == 0;
    if (solver->unexpected_calls != unexpected_before)
    {
      out.exception = true;
      out.exception_text = "UNEXPECTED_SEARCH_SIGNATURE";
    }
    for (auto iterator = solver->calls.rbegin(); iterator != solver->calls.rend(); ++iterator)
    {
      if (iterator->trial == trial && iterator->phase == "PATH")
      {
        out.last_path_known = true;
        out.last_path_success = iterator->success;
        out.last_path_timeout_s = iterator->timeout_s;
        break;
      }
    }
    trajectory.clear();
    out.stop_class = classify_stop(out);
  };

  solver->reset_counts();
  solver->record_detail = true;
  Trial preflight;
  preflight.index = -1;
  preflight.record_detail = true;
  const int preflight_limit = 5;
  int preflight_attempts = 0;
  for (int attempt = 0; attempt < preflight_limit; ++attempt)
  {
    solver->reset_counts();
    preflight = Trial();
    preflight.index = -1;
    preflight.record_detail = true;
    run_once(-1, true, preflight);
    ++preflight_attempts;
    if (preflight.start_ik_success)
    {
      break;
    }
  }
  const bool wrapper_miss = preflight.start_ik_success && preflight.path_calls == 0;
  const bool preservation_failed = solver->unexpected_calls != 0 || solver->nonempty_limits_calls != 0 || preflight.exception;
  const bool timeout_unchanged = std::abs(group->getDefaultIKTimeout() - group_timeout_before) < 1e-15;
  const bool preflight_ok = preflight.start_ik_success && preflight.start_calls > 0 && preflight.path_calls > 0 &&
                            !preservation_failed && timeout_unchanged;
  const int preflight_start_calls = preflight.start_calls;
  const int preflight_path_calls = preflight.path_calls;
  const double preflight_fraction = preflight.fraction;
  const bool preflight_start_ok = preflight.start_ik_success;
  double preflight_path_timeout = std::numeric_limits<double>::quiet_NaN();
  for (const auto & call : solver->calls)
  {
    if (call.phase == "PATH")
    {
      preflight_path_timeout = call.timeout_s;
      break;
    }
  }
  std::cout << "PREFLIGHT start_calls " << preflight_start_calls << " path_calls " << preflight_path_calls
            << " unexpected " << solver->unexpected_calls << " nonempty_limits " << solver->nonempty_limits_calls
            << " ok " << (preflight_ok ? "YES" : "NO") << "\n"
            << std::flush;

  std::vector<Trial> trials;
  if (preflight_ok)
  {
    solver->reset_counts();
    for (int index = 0; index < 20; ++index)
    {
      const bool detail = index >= 10;
      Trial trial;
      trial.index = index;
      trial.record_detail = detail;
      const int call_begin = static_cast<int>(solver->calls.size());
      run_once(index, detail, trial);
      if (solver->unexpected_calls != 0 || solver->nonempty_limits_calls != 0)
      {
        trial.exception = true;
        trial.exception_text = "WRAPPER_PRESERVATION_FAILED";
        trial.stop_class = "UNCONFIRMED";
        trials.push_back(trial);
        std::cout << "TRIAL " << index << " PRESERVATION_FAILED\n" << std::flush;
        break;
      }
      (void)call_begin;
      trials.push_back(trial);
      std::cout << "TRIAL " << index << " RECORD " << (detail ? "ON" : "OFF") << " START "
                << (trial.start_ik_success ? "OK" : "FAIL") << " FRACTION ";
      if (std::isfinite(trial.fraction))
      {
        std::cout << std::setprecision(17) << trial.fraction;
      }
      else
      {
        std::cout << "null";
      }
      std::cout << "\n" << std::flush;
    }
  }

  const double group_timeout_after = group->getDefaultIKTimeout();
  std::ofstream output(argv[4]);
  output << std::setprecision(17);
  output << "{\n  \"gate\": \"P-022-AB\",\n  \"experiment\": \"LIBRARY_PATH_TRACE\",\n"
         << "  \"reviewer_closed\": false,\n  \"p022_reopened\": false,\n"
         << "  \"execution_order\": [\n"
         << "    \"PREFLIGHT_UP_TO_5_UNTIL_START_IK_NOT_IN_THE_20\",\n"
         << "    \"TRIALS_0_TO_9_RECORD_DETAIL_OFF\",\n"
         << "    \"TRIALS_10_TO_19_RECORD_DETAIL_ON\"\n  ],\n"
         << "  \"set_default_ik_timeout\": false,\n  \"start_ik_timeout_argument_s\": ";
  write_number(output, start_timeout);
  output << ",\n  \"group_default_ik_timeout_before_s\": ";
  write_number(output, group_timeout_before);
  output << ",\n  \"group_default_ik_timeout_after_s\": ";
  write_number(output, group_timeout_after);
  output << ",\n  \"search_discretization\": ";
  write_number(output, search_discretization);
  output << ",\n  \"solver\": \"KDLKinematicsPlugin\",\n  \"wrapper_calls_parent\": true,\n"
         << "  \"eef_step_m\": 0.005,\n  \"eef_rotation_step_rad\": 0,\n"
         << "  \"preflight_passed\": " << (preflight_ok ? "true" : "false")
         << ",\n  \"preflight_attempts\": " << preflight_attempts
         << ",\n  \"preflight_wrapper_miss\": " << (wrapper_miss ? "true" : "false")
         << ",\n  \"preflight_start_ik_success\": " << (preflight_start_ok ? "true" : "false")
         << ",\n  \"preflight_start_calls\": " << preflight_start_calls
         << ",\n  \"preflight_path_calls\": " << preflight_path_calls
         << ",\n  \"preflight_fraction\": ";
  write_number(output, preflight_fraction);
  output << ",\n  \"preflight_first_path_timeout_s\": ";
  write_number(output, preflight_path_timeout);
  output << ",\n  \"preflight_unexpected_signature_calls\": " << (preflight_ok ? 0 : solver->unexpected_calls)
         << ",\n  \"preflight_nonempty_limits_calls\": " << (preflight_ok ? 0 : solver->nonempty_limits_calls)
         << ",\n  \"main_experiment_run\": " << (preflight_ok ? "true" : "false")
         << ",\n  \"trials\": [\n";
  for (std::size_t index = 0; index < trials.size(); ++index)
  {
    const auto & trial = trials[index];
    if (index)
    {
      output << ",\n";
    }
    output << "    {\"trial\": " << trial.index << ", \"record_detail\": " << (trial.record_detail ? "true" : "false")
           << ", \"start_ik_success\": " << (trial.start_ik_success ? "true" : "false")
           << ", \"start_ik_elapsed_s\": ";
    write_number(output, trial.start_elapsed_s);
    output << ", \"start_wrapper_calls\": " << trial.start_calls << ", \"path_wrapper_calls\": " << trial.path_calls
           << ", \"path_instrumentation_miss\": " << (trial.path_miss ? "true" : "false")
           << ", \"cartesian_fraction\": ";
    write_number(output, trial.fraction);
    output << ", \"matches_20_over_33\": "
           << (matches_20_33(trial.fraction, static_cast<int>(poses.size())) ? "true" : "false")
           << ", \"cartesian_elapsed_s\": ";
    write_number(output, trial.path_elapsed_s);
    output << ", \"last_path_ik_success\": ";
    if (!trial.last_path_known)
    {
      output << "null";
    }
    else
    {
      output << (trial.last_path_success ? "true" : "false");
    }
    output << ", \"last_path_timeout_s\": ";
    write_number(output, trial.last_path_timeout_s);
    output << ", \"stop_class\": \"" << trial.stop_class << "\", \"call_site\": \"UNCONFIRMED\""
           << ", \"applies_to_past_library_runs\": false, \"error\": ";
    if (trial.exception_text.empty())
    {
      output << "null";
    }
    else
    {
      output << "\"" << trial.exception_text << "\"";
    }
    output << "}";
  }
  output << "\n  ],\n  \"calls\": [\n";
  bool first_call = true;
  for (const auto & call : solver->calls)
  {
    if (call.trial < 0)
    {
      continue;
    }
    if (!first_call)
    {
      output << ",\n";
    }
    first_call = false;
    output << "    {\"trial\": " << call.trial << ", \"phase\": \"" << call.phase << "\", \"timeout_s\": ";
    write_number(output, call.timeout_s);
    output << ", \"elapsed_s\": ";
    write_number(output, call.elapsed_s);
    output << ", \"ik_success\": " << (call.success ? "true" : "false") << ", \"error_code\": " << call.error_code
           << ", \"consistency_limits_empty\": " << (call.limits_empty ? "true" : "false")
           << ", \"detail\": " << (call.detail ? "true" : "false");
    if (call.detail)
    {
      output << ", \"target_position_m\": ";
      write_vector(output, call.position);
      output << ", \"target_quaternion_xyzw\": ";
      write_vector(output, call.quaternion_xyzw);
      output << ", \"seed\": ";
      write_vector(output, call.seed);
      output << ", \"solution\": ";
      if (call.solution.empty())
      {
        output << "null";
      }
      else
      {
        write_vector(output, call.solution);
      }
    }
    output << "}";
  }
  output << "\n  ]\n}\n";
  std::cout << "MAIN_EXPERIMENT " << (preflight_ok ? "RUN" : "STOPPED") << " trials " << trials.size() << "\n";
  rclcpp::shutdown();
  return preflight_ok ? 0 : 4;
}
