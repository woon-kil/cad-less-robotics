// UR-ANALYTIC-IK-PROCESS-CONNECT. PROCESS run별 사다리 탐색과 고정 표본 FK.
// 시작 관절은 자유다. 충돌과 시간 매개변수화는 평가하지 않는다.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <descartes_light/core/waypoint_sampler.h>
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>
#include <descartes_light/solvers/ladder_graph/ladder_graph_solver.h>

namespace
{
const char * kJointNames[6] = {
  "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
const int kRunCount = 21;
const int kCandidateCount = 256;
const int kSampleDenominator = 100;
const double kTranslationMaxM = 0.001;
const double kRotationMaxRad = 0.01;
const double kRunWallS = 600.0;

struct Candidate
{
  int source_code = 0;
  int kinematic_class_id = 0;
  Eigen::VectorXd joints;
};

struct Waypoint
{
  int sequence_id = -1;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  std::vector<Candidate> candidates;
};

using Clock = std::chrono::steady_clock;

class RecordedJointSampler : public descartes_light::WaypointSampler<double>
{
public:
  explicit RecordedJointSampler(std::vector<descartes_light::State<double>::ConstPtr> states)
  : states_(std::move(states))
  {
  }

  std::vector<descartes_light::StateSample<double>> sample() const override
  {
    std::vector<descartes_light::StateSample<double>> samples;
    samples.reserve(states_.size());
    for (const auto & state : states_)
    {
      samples.emplace_back(state, 0.0);
    }
    return samples;
  }

private:
  std::vector<descartes_light::State<double>::ConstPtr> states_;
};

std::string read_file(const std::string & path)
{
  std::ifstream input(path);
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
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

double rotation_error_rad(const Eigen::Matrix3d & target, const Eigen::Matrix3d & actual)
{
  const Eigen::Matrix3d relative = target.transpose() * actual;
  double cosine = (relative.trace() - 1.0) * 0.5;
  if (cosine > 1.0)
  {
    cosine = 1.0;
  }
  if (cosine < -1.0)
  {
    cosine = -1.0;
  }
  return std::acos(cosine);
}

template <typename T>
T read_pod(std::istream & input)
{
  T value{};
  input.read(reinterpret_cast<char *>(&value), sizeof(T));
  return value;
}

bool load_runs(const std::string & path, std::vector<std::vector<Waypoint>> & runs)
{
  std::ifstream input(path, std::ios::binary);
  char magic[5] = {};
  input.read(magic, 5);
  if (!input || std::string(magic, 5) != "URPC1")
  {
    return false;
  }
  const auto run_count = read_pod<std::uint32_t>(input);
  if (run_count != static_cast<std::uint32_t>(kRunCount))
  {
    return false;
  }
  runs.resize(kRunCount);
  int process_count = 0;
  for (int run_id = 0; run_id < kRunCount; ++run_id)
  {
    const auto stored_run = read_pod<std::int32_t>(input);
    const auto waypoint_count = read_pod<std::uint32_t>(input);
    if (!input || stored_run != run_id || waypoint_count == 0)
    {
      return false;
    }
    runs[static_cast<std::size_t>(run_id)].resize(waypoint_count);
    for (std::uint32_t waypoint_index = 0; waypoint_index < waypoint_count; ++waypoint_index)
    {
      Waypoint & waypoint = runs[static_cast<std::size_t>(run_id)][waypoint_index];
      waypoint.sequence_id = read_pod<std::int32_t>(input);
      double position[3];
      double orientation[4];
      input.read(reinterpret_cast<char *>(position), sizeof(position));
      input.read(reinterpret_cast<char *>(orientation), sizeof(orientation));
      waypoint.position = Eigen::Vector3d(position[0], position[1], position[2]);
      waypoint.orientation = Eigen::Quaterniond(orientation[3], orientation[0], orientation[1], orientation[2]);
      const auto candidate_count = read_pod<std::uint32_t>(input);
      if (!input || candidate_count != static_cast<std::uint32_t>(kCandidateCount))
      {
        return false;
      }
      waypoint.candidates.resize(kCandidateCount);
      for (int candidate_index = 0; candidate_index < kCandidateCount; ++candidate_index)
      {
        Candidate & candidate = waypoint.candidates[static_cast<std::size_t>(candidate_index)];
        candidate.kinematic_class_id = read_pod<std::int32_t>(input);
        candidate.source_code = read_pod<std::uint8_t>(input);
        double joints[6];
        input.read(reinterpret_cast<char *>(joints), sizeof(joints));
        if (!input || (candidate.source_code != 0 && candidate.source_code != 1))
        {
          return false;
        }
        candidate.joints = Eigen::VectorXd(6);
        for (int joint_index = 0; joint_index < 6; ++joint_index)
        {
          if (!std::isfinite(joints[joint_index]))
          {
            return false;
          }
          candidate.joints[joint_index] = joints[joint_index];
        }
      }
      if (!waypoint.position.allFinite() || !std::isfinite(waypoint.orientation.norm()) ||
          waypoint.orientation.norm() <= 0.0)
      {
        return false;
      }
      ++process_count;
    }
  }
  return process_count == 721 && input && input.peek() == EOF;
}

const char * source_name(int source_code)
{
  return source_code == 0 ? "solver" : "redundant_2pi";
}

Eigen::Isometry3d tool_in_base(moveit::core::RobotState & state)
{
  return state.getGlobalLinkTransform("base_link").inverse() * state.getGlobalLinkTransform("tool0");
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 5)
  {
    std::cerr << "usage: ur_analytic_ik_process_connect urdf srdf runs.bin out.json\n";
    return 2;
  }
  std::vector<std::vector<Waypoint>> runs;
  if (!load_runs(argv[3], runs))
  {
    std::cerr << "BLOCKED_INPUT_INTEGRITY runs\n";
    return 2;
  }

  rclcpp::init(argc, argv);
  auto urdf = urdf::parseURDF(read_file(argv[1]));
  auto srdf = std::make_shared<srdf::Model>();
  if (!urdf || !srdf->initString(*urdf, read_file(argv[2])))
  {
    std::cerr << "FAIL_ROBOT_MODEL_LOAD\n";
    return 3;
  }
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  auto * group = model->getJointModelGroup("ur_manipulator");
  if (group == nullptr || model->getLinkModel("base_link") == nullptr || model->getLinkModel("tool0") == nullptr)
  {
    std::cerr << "FAIL_ROBOT_MODEL_LOAD\n";
    return 3;
  }
  const auto joint_names = group->getActiveJointModelNames();
  if (joint_names.size() != 6)
  {
    std::cerr << "FAIL_JOINT_COUNT\n";
    return 3;
  }
  for (int index = 0; index < 6; ++index)
  {
    if (joint_names[static_cast<std::size_t>(index)] != kJointNames[index])
    {
      std::cerr << "FAIL_JOINT_ORDER\n";
      return 3;
    }
  }
  moveit::core::RobotState state(model);
  const int threads = std::max(1, static_cast<int>(std::thread::hardware_concurrency()));

  std::ofstream output(argv[4]);
  output << std::setprecision(17);
  output << "{\n  \"gate\": \"UR-ANALYTIC-IK-PROCESS-CONNECT\",\n"
         << "  \"solver\": \"descartes_light LadderGraphSolverD\",\n"
         << "  \"edge_evaluator\": \"EuclideanDistanceEdgeEvaluatorD\",\n"
         << "  \"edge_scale\": \"NONE\",\n"
         << "  \"angle_wrapping\": false,\n"
         << "  \"start_state\": \"FREE_OFFLINE\",\n"
         << "  \"approach_from_current_state\": \"NOT_CLAIMED\",\n"
         << "  \"collision\": \"NOT_EVALUATED\",\n"
         << "  \"time_parameterization\": \"NOT_EVALUATED\",\n"
         << "  \"transition\": \"NOT_IN_THIS_STAGE\",\n"
         << "  \"sample_t\": \"k/100\",\n"
         << "  \"sample_count\": 101,\n"
         << "  \"translation_max_m\": 0.001,\n"
         << "  \"rotation_max_rad\": 0.01,\n"
         << "  \"whole_interval_guarantee\": false,\n"
         << "  \"moveit_cartesian_precision_equivalent\": false,\n"
         << "  \"threads\": " << threads << ",\n"
         << "  \"per_run_wall_s\": 600,\n"
         << "  \"retry\": 0,\n"
         << "  \"runs\": [\n";

  for (int run_id = 0; run_id < kRunCount; ++run_id)
  {
    const auto started = Clock::now();
    const auto elapsed = [&started]() {
      return std::chrono::duration<double>(Clock::now() - started).count();
    };
    const std::vector<Waypoint> & waypoints = runs[static_cast<std::size_t>(run_id)];
    std::vector<std::vector<descartes_light::State<double>::ConstPtr>> states(waypoints.size());
    std::vector<descartes_light::WaypointSampler<double>::ConstPtr> samplers;
    samplers.reserve(waypoints.size());
    for (std::size_t waypoint_index = 0; waypoint_index < waypoints.size(); ++waypoint_index)
    {
      states[waypoint_index].reserve(static_cast<std::size_t>(kCandidateCount));
      for (const Candidate & candidate : waypoints[waypoint_index].candidates)
      {
        states[waypoint_index].push_back(std::make_shared<descartes_light::State<double>>(candidate.joints));
      }
      samplers.push_back(std::make_shared<RecordedJointSampler>(states[waypoint_index]));
    }
    auto edge = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<double>>();
    descartes_light::LadderGraphSolver<double> solver(threads);
    const descartes_light::BuildStatus built = solver.build(
      samplers, {edge}, {});

    std::string search_status = "NO_PATH";
    std::string accuracy_status = "NOT_EVALUATED";
    bool have_path = false;
    double ladder_cost = std::numeric_limits<double>::quiet_NaN();
    std::vector<std::size_t> chosen;
    if (elapsed() > kRunWallS)
    {
      search_status = "TIMEOUT";
    }
    else if (built)
    {
      try
      {
        const descartes_light::SearchResult<double> result = solver.search();
        if (result.trajectory.size() != waypoints.size())
        {
          std::cerr << "FAIL_PATH_LENGTH " << run_id << "\n";
          return 6;
        }
        chosen.resize(waypoints.size());
        for (std::size_t waypoint_index = 0; waypoint_index < waypoints.size(); ++waypoint_index)
        {
          const auto & found = result.trajectory[waypoint_index];
          auto match = std::find(
            states[waypoint_index].begin(), states[waypoint_index].end(), found);
          if (match == states[waypoint_index].end())
          {
            std::cerr << "FAIL_PATH_IDENTITY " << run_id << "\n";
            return 6;
          }
          chosen[waypoint_index] = static_cast<std::size_t>(match - states[waypoint_index].begin());
        }
        ladder_cost = result.cost;
        have_path = true;
        search_status = "FOUND";
      }
      catch (const std::runtime_error &)
      {
        search_status = "NO_PATH";
      }
    }
    if (elapsed() > kRunWallS)
    {
      search_status = "TIMEOUT";
      have_path = false;
      accuracy_status = "NOT_EVALUATED";
    }

    double max_position = -1.0;
    double max_orientation = -1.0;
    int max_position_start = -1;
    int max_position_end = -1;
    double max_position_t = std::numeric_limits<double>::quiet_NaN();
    int max_orientation_start = -1;
    int max_orientation_end = -1;
    double max_orientation_t = std::numeric_limits<double>::quiet_NaN();
    bool sampled_fail = false;
    bool sample_complete = false;
    if (have_path && search_status == "FOUND")
    {
      sample_complete = true;
      for (std::size_t segment = 0; segment + 1 < waypoints.size() && sample_complete; ++segment)
      {
        const Waypoint & start = waypoints[segment];
        const Waypoint & end = waypoints[segment + 1];
        const Eigen::VectorXd & start_joints = start.candidates[chosen[segment]].joints;
        const Eigen::VectorXd & end_joints = end.candidates[chosen[segment + 1]].joints;
        for (int sample_index = 0; sample_index <= kSampleDenominator; ++sample_index)
        {
          if (elapsed() > kRunWallS)
          {
            search_status = "TIMEOUT";
            accuracy_status = "NOT_EVALUATED";
            sample_complete = false;
            break;
          }
          const double fraction = static_cast<double>(sample_index) / static_cast<double>(kSampleDenominator);
          const Eigen::VectorXd joints = (1.0 - fraction) * start_joints + fraction * end_joints;
          std::vector<double> query(6);
          for (int joint_index = 0; joint_index < 6; ++joint_index)
          {
            query[static_cast<std::size_t>(joint_index)] = joints[joint_index];
          }
          state.setJointGroupPositions(group, query);
          state.update();
          const Eigen::Isometry3d actual = tool_in_base(state);
          const Eigen::Vector3d target_position = (1.0 - fraction) * start.position + fraction * end.position;
          const Eigen::Quaterniond target_orientation = start.orientation.slerp(fraction, end.orientation);
          const double position_error = (actual.translation() - target_position).norm();
          const double orientation_error = rotation_error_rad(
            target_orientation.toRotationMatrix(), actual.rotation());
          if (position_error > max_position)
          {
            max_position = position_error;
            max_position_start = start.sequence_id;
            max_position_end = end.sequence_id;
            max_position_t = fraction;
          }
          if (orientation_error > max_orientation)
          {
            max_orientation = orientation_error;
            max_orientation_start = start.sequence_id;
            max_orientation_end = end.sequence_id;
            max_orientation_t = fraction;
          }
          if (position_error > kTranslationMaxM || orientation_error > kRotationMaxRad)
          {
            sampled_fail = true;
          }
        }
      }
      if (sample_complete)
      {
        accuracy_status = sampled_fail ? "SAMPLED_ACCURACY_FAIL" : "SAMPLED_ACCURACY_PASS";
      }
    }

    if (run_id)
    {
      output << ",\n";
    }
    output << "    {\"run_id\": " << run_id
           << ", \"waypoint_count\": " << waypoints.size()
           << ", \"search\": \"" << search_status << "\""
           << ", \"process_accuracy\": \"" << accuracy_status << "\""
           << ", \"collision\": \"NOT_EVALUATED\""
           << ", \"time_parameterization\": \"NOT_EVALUATED\""
           << ", \"result_role\": \""
           << (search_status == "FOUND" ? "GEOMETRIC_CANDIDATE_PATH" : "NONE") << "\""
           << ", \"ladder_cost\": ";
    write_number(output, ladder_cost);
    output << ", \"elapsed_s\": ";
    write_number(output, elapsed());
    output << ", \"max_position_error_m\": ";
    write_number(output, max_position);
    output << ", \"max_position_sequence_ids\": [" << max_position_start << ", " << max_position_end << "]"
           << ", \"max_position_t\": ";
    write_number(output, max_position_t);
    output << ", \"max_orientation_error_rad\": ";
    write_number(output, max_orientation);
    output << ", \"max_orientation_sequence_ids\": [" << max_orientation_start << ", " << max_orientation_end << "]"
           << ", \"max_orientation_t\": ";
    write_number(output, max_orientation_t);
    output << ", \"selected\": [";
    if (search_status == "FOUND")
    {
      for (std::size_t waypoint_index = 0; waypoint_index < waypoints.size(); ++waypoint_index)
      {
        if (waypoint_index)
        {
          output << ", ";
        }
        const Candidate & candidate = waypoints[waypoint_index].candidates[chosen[waypoint_index]];
        output << "{\"sequence_id\": " << waypoints[waypoint_index].sequence_id
               << ", \"candidate_index\": " << chosen[waypoint_index]
               << ", \"source\": \"" << source_name(candidate.source_code) << "\""
               << ", \"kinematic_class_id\": " << candidate.kinematic_class_id
               << ", \"joints_rad\": [";
        for (int joint_index = 0; joint_index < 6; ++joint_index)
        {
          if (joint_index)
          {
            output << ", ";
          }
          write_number(output, candidate.joints[joint_index]);
        }
        output << "]}";
      }
    }
    output << "]}";
    output.flush();
    std::cout << "RUN " << run_id << " " << search_status << " " << accuracy_status
              << " elapsed " << elapsed() << "\n";
  }
  output << "\n  ]\n}\n";
  rclcpp::shutdown();
  return 0;
}
