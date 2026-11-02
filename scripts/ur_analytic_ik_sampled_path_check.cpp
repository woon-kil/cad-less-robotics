// 저장된 관절열의 원본 구간 t=k/100을 다시 계산한다. 탐색은 하지 않는다.

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

namespace
{
const int kSampleDenominator = 100;
const double kPositionMaxM = 0.001;
const double kOrientationMaxRad = 0.01;

struct Segment
{
  int start_id = -1;
  int end_id = -1;
  Eigen::Vector3d start_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d end_position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond start_orientation = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond end_orientation = Eigen::Quaterniond::Identity();
  Eigen::VectorXd start_joints;
  Eigen::VectorXd end_joints;
  Eigen::VectorXd mid_joints;
  bool piecewise = false;
};

struct Run
{
  int run_id = -1;
  bool piecewise = false;
  std::vector<Segment> segments;
};

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

Eigen::Isometry3d tool_in_base(const moveit::core::RobotState & state)
{
  return state.getGlobalLinkTransform("base_link").inverse() * state.getGlobalLinkTransform("tool0");
}

std::string read_file(const std::string & path)
{
  std::ifstream input(path);
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

template <typename T>
bool read_pod(std::istream & input, T * value)
{
  input.read(reinterpret_cast<char *>(value), sizeof(T));
  return static_cast<bool>(input);
}

bool read_vector(std::istream & input, Eigen::VectorXd * value, int width)
{
  value->resize(width);
  for (int index = 0; index < width; ++index)
  {
    double component = 0.0;
    if (!read_pod(input, &component))
    {
      return false;
    }
    (*value)[index] = component;
  }
  return true;
}

bool load_runs(const std::string & path, std::vector<Run> * runs)
{
  std::ifstream input(path, std::ios::binary);
  char magic[4] = {};
  input.read(magic, 4);
  if (!input || std::string(magic, 4) != "CHK1")
  {
    return false;
  }
  std::uint32_t run_count = 0;
  if (!read_pod(input, &run_count) || run_count != 21)
  {
    return false;
  }
  runs->resize(run_count);
  for (std::uint32_t index = 0; index < run_count; ++index)
  {
    Run & run = (*runs)[index];
    std::uint8_t piecewise = 0;
    std::uint32_t segment_count = 0;
    if (!read_pod(input, &run.run_id) || !read_pod(input, &piecewise) || !read_pod(input, &segment_count))
    {
      return false;
    }
    run.piecewise = piecewise == 1;
    run.segments.resize(segment_count);
    for (std::uint32_t segment_index = 0; segment_index < segment_count; ++segment_index)
    {
      Segment & segment = run.segments[segment_index];
      segment.piecewise = run.piecewise;
      double start_position[3];
      double end_position[3];
      double start_orientation[4];
      double end_orientation[4];
      if (!read_pod(input, &segment.start_id) || !read_pod(input, &segment.end_id) ||
          !input.read(reinterpret_cast<char *>(start_position), sizeof(start_position)) ||
          !input.read(reinterpret_cast<char *>(end_position), sizeof(end_position)) ||
          !input.read(reinterpret_cast<char *>(start_orientation), sizeof(start_orientation)) ||
          !input.read(reinterpret_cast<char *>(end_orientation), sizeof(end_orientation)) ||
          !read_vector(input, &segment.start_joints, 6) || !read_vector(input, &segment.end_joints, 6))
      {
        return false;
      }
      if (run.piecewise && !read_vector(input, &segment.mid_joints, 6))
      {
        return false;
      }
      segment.start_position = Eigen::Vector3d(start_position[0], start_position[1], start_position[2]);
      segment.end_position = Eigen::Vector3d(end_position[0], end_position[1], end_position[2]);
      segment.start_orientation = Eigen::Quaterniond(
        start_orientation[3], start_orientation[0], start_orientation[1], start_orientation[2]);
      segment.end_orientation = Eigen::Quaterniond(
        end_orientation[3], end_orientation[0], end_orientation[1], end_orientation[2]);
    }
  }
  return static_cast<bool>(input) && input.peek() == EOF;
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

struct SampleResult
{
  bool finite = false;
  double position_error = std::numeric_limits<double>::quiet_NaN();
  double orientation_error = std::numeric_limits<double>::quiet_NaN();
};

SampleResult evaluate_sample(
  const Segment & segment,
  int sample_index,
  moveit::core::RobotState * state,
  const moveit::core::JointModelGroup * group)
{
  SampleResult result;
  const double parameter = static_cast<double>(sample_index) / static_cast<double>(kSampleDenominator);
  Eigen::VectorXd joints;
  if (segment.piecewise)
  {
    if (parameter <= 0.5)
    {
      const double local = parameter / 0.5;
      joints = (1.0 - local) * segment.start_joints + local * segment.mid_joints;
    }
    else
    {
      const double local = (parameter - 0.5) / 0.5;
      joints = (1.0 - local) * segment.mid_joints + local * segment.end_joints;
    }
  }
  else
  {
    joints = (1.0 - parameter) * segment.start_joints + parameter * segment.end_joints;
  }
  const Eigen::Vector3d target_position =
    (1.0 - parameter) * segment.start_position + parameter * segment.end_position;
  const bool quaternion_usable = segment.start_orientation.coeffs().allFinite() &&
                                 segment.end_orientation.coeffs().allFinite() &&
                                 segment.start_orientation.norm() > 0.0 && segment.end_orientation.norm() > 0.0;
  const Eigen::Quaterniond start_unit = quaternion_usable ? segment.start_orientation.normalized()
                                                          : Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond end_unit = quaternion_usable ? segment.end_orientation.normalized()
                                                        : Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond target_orientation = quaternion_usable ? start_unit.slerp(parameter, end_unit)
                                                                  : Eigen::Quaterniond::Identity();
  const Eigen::Matrix3d target_rotation = target_orientation.toRotationMatrix();
  if (!joints.allFinite() || !target_position.allFinite() || !quaternion_usable ||
      !target_orientation.coeffs().allFinite() || !target_rotation.allFinite())
  {
    return result;
  }
  std::vector<double> query(joints.data(), joints.data() + joints.size());
  state->setJointGroupPositions(group, query);
  state->update();
  const Eigen::Isometry3d actual = tool_in_base(*state);
  if (!actual.translation().allFinite() || !actual.rotation().allFinite())
  {
    return result;
  }
  result.position_error = (actual.translation() - target_position).norm();
  result.orientation_error = rotation_error_rad(target_rotation, actual.rotation());
  result.finite = std::isfinite(result.position_error) && std::isfinite(result.orientation_error);
  return result;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 5)
  {
    std::cerr << "usage: ur_analytic_ik_sampled_path_check urdf srdf input.bin out.json\n";
    return 2;
  }
  std::vector<Run> runs;
  if (!load_runs(argv[3], &runs))
  {
    std::cerr << "BLOCKED_INPUT_INTEGRITY input\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  auto urdf = urdf::parseURDF(read_file(argv[1]));
  auto srdf = std::make_shared<srdf::Model>();
  if (!urdf || !srdf->initString(*urdf, read_file(argv[2])))
  {
    std::cerr << "EXECUTION_ERROR robot\n";
    return 4;
  }
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  auto * group = model->getJointModelGroup("ur_manipulator");
  if (group == nullptr)
  {
    std::cerr << "EXECUTION_ERROR robot\n";
    return 4;
  }
  moveit::core::RobotState state(model);
  std::ofstream output(argv[4]);
  output << std::setprecision(17);
  output << "{\n  \"gate\": \"UR-ANALYTIC-IK-PROCESS-SAMPLED-PATH-CHECK\",\n"
         << "  \"role\": \"INDEPENDENT_RESAMPLE\",\n  \"search\": false,\n"
         << "  \"translation_max_m\": 0.001,\n  \"rotation_max_rad\": 0.01,\n  \"runs\": [\n";
  bool all_pass = true;
  for (std::size_t run_index = 0; run_index < runs.size(); ++run_index)
  {
    const Run & run = runs[run_index];
    const int expected = static_cast<int>(run.segments.size()) * (kSampleDenominator + 1);
    int actual = 0;
    int nonfinite = 0;
    int violations = 0;
    double max_position = std::numeric_limits<double>::quiet_NaN();
    double max_orientation = std::numeric_limits<double>::quiet_NaN();
    int max_position_start = -1;
    int max_position_end = -1;
    double max_position_t = std::numeric_limits<double>::quiet_NaN();
    int max_orientation_start = -1;
    int max_orientation_end = -1;
    double max_orientation_t = std::numeric_limits<double>::quiet_NaN();
    for (const Segment & segment : run.segments)
    {
      for (int sample_index = 0; sample_index <= kSampleDenominator; ++sample_index)
      {
        ++actual;
        const SampleResult sample = evaluate_sample(segment, sample_index, &state, group);
        const double parameter = static_cast<double>(sample_index) / static_cast<double>(kSampleDenominator);
        if (!sample.finite)
        {
          ++nonfinite;
          continue;
        }
        if (sample.position_error > max_position || !std::isfinite(max_position))
        {
          max_position = sample.position_error;
          max_position_start = segment.start_id;
          max_position_end = segment.end_id;
          max_position_t = parameter;
        }
        if (sample.orientation_error > max_orientation || !std::isfinite(max_orientation))
        {
          max_orientation = sample.orientation_error;
          max_orientation_start = segment.start_id;
          max_orientation_end = segment.end_id;
          max_orientation_t = parameter;
        }
        if (!(sample.position_error <= kPositionMaxM) || !(sample.orientation_error <= kOrientationMaxRad))
        {
          ++violations;
        }
      }
    }
    const bool passed = actual == expected && nonfinite == 0 && violations == 0;
    if (!passed)
    {
      all_pass = false;
    }
    if (run_index)
    {
      output << ",\n";
    }
    output << "    {\"run_id\": " << run.run_id
           << ", \"piecewise\": " << (run.piecewise ? "true" : "false")
           << ", \"expected_check_count\": " << expected
           << ", \"actual_check_count\": " << actual
           << ", \"nonfinite_count\": " << nonfinite
           << ", \"violation_count\": " << violations
           << ", \"max_position_error_m\": ";
    write_number(output, max_position);
    output << ", \"max_position_sequence_ids\": [" << max_position_start << ", " << max_position_end
           << "], \"max_position_t\": ";
    write_number(output, max_position_t);
    output << ", \"max_orientation_error_rad\": ";
    write_number(output, max_orientation);
    output << ", \"max_orientation_sequence_ids\": [" << max_orientation_start << ", " << max_orientation_end
           << "], \"max_orientation_t\": ";
    write_number(output, max_orientation_t);
    output << ", \"pass\": " << (passed ? "true" : "false") << "}";
    std::cout << "RUN " << run.run_id << (passed ? " PASS" : " FAIL")
              << " checks " << actual << "/" << expected
              << " nonfinite " << nonfinite << " violations " << violations << "\n";
  }
  output << "\n  ],\n  \"all_pass\": " << (all_pass ? "true" : "false") << ",\n  \"complete\": true\n}\n";
  rclcpp::shutdown();
  return all_pass ? 0 : 1;
}
