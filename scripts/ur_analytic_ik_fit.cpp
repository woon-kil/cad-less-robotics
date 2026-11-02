// UR-ANALYTIC-IK-FIT. 64개 관절 표본의 모델 적합성만 본다.
// 741 waypoint와 Descartes는 호출하지 않는다. /opt/ros/jazzy 는 수정하지 않는다.

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <tesseract/kinematics/ur/ur_inv_kin.h>
#include <tesseract/kinematics/utils.h>

namespace
{
const char * kJointNames[6] = {
  "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
const int kSampleCount = 64;
const std::uint32_t kSampleSeed = 20261102;
const double kMarginRad = 0.05;
const double kPositionMaxM = 1.0e-6;
const double kOrientationMaxRad = 1.0e-6;
const double kDiagnosticMinSv = 1.0e-3;
const double kLimitBoundaryRad = 1.0e-12;

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

bool within_limits(const Eigen::VectorXd & joints, const Eigen::MatrixX2d & limits)
{
  for (int index = 0; index < joints.size(); ++index)
  {
    if (!std::isfinite(joints[index]))
    {
      return false;
    }
    if (joints[index] < limits(index, 0) - kLimitBoundaryRad || joints[index] > limits(index, 1) + kLimitBoundaryRad)
    {
      return false;
    }
  }
  return true;
}

Eigen::Isometry3d tool_in_base(const moveit::core::RobotState & state)
{
  return state.getGlobalLinkTransform("base_link").inverse() * state.getGlobalLinkTransform("tool0");
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 4)
  {
    std::cerr << "usage: ur_analytic_ik_fit urdf srdf output\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ur_analytic_ik_fit");
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
    std::cerr << "FAIL_JOINT_COUNT " << joint_names.size() << "\n";
    return 3;
  }
  for (int index = 0; index < 6; ++index)
  {
    if (joint_names[static_cast<std::size_t>(index)] != kJointNames[index])
    {
      std::cerr << "FAIL_JOINT_ORDER " << joint_names[static_cast<std::size_t>(index)] << "\n";
      return 3;
    }
  }

  Eigen::MatrixX2d limits(6, 2);
  for (int index = 0; index < 6; ++index)
  {
    const auto & bounds = group->getActiveJointModels()[static_cast<std::size_t>(index)]->getVariableBounds();
    if (bounds.size() != 1 || !bounds[0].position_bounded_)
    {
      std::cerr << "FAIL_JOINT_LIMIT " << kJointNames[index] << "\n";
      return 3;
    }
    limits(index, 0) = bounds[0].min_position_;
    limits(index, 1) = bounds[0].max_position_;
    if (!(limits(index, 1) - limits(index, 0) > 2.0 * kMarginRad))
    {
      std::cerr << "FAIL_SAMPLE_MARGIN " << kJointNames[index] << "\n";
      return 3;
    }
  }

  std::vector<tesseract::common::JointId> joint_ids;
  for (const char * name : kJointNames)
  {
    joint_ids.emplace_back(name);
  }
  const tesseract::common::LinkId base_link("base_link");
  const tesseract::common::LinkId tip_link("tool0");
  tesseract::kinematics::URInvKin solver(
    tesseract::kinematics::UR5eParameters, base_link, tip_link, joint_ids);
  const std::vector<Eigen::Index> redundant_joints{0, 1, 2, 3, 4, 5};

  std::mt19937 generator(kSampleSeed);
  std::vector<std::array<double, 6>> samples(kSampleCount);
  for (int sample_index = 0; sample_index < kSampleCount; ++sample_index)
  {
    for (int joint_index = 0; joint_index < 6; ++joint_index)
    {
      std::uniform_real_distribution<double> distribution(
        limits(joint_index, 0) + kMarginRad, limits(joint_index, 1) - kMarginRad);
      samples[static_cast<std::size_t>(sample_index)][static_cast<std::size_t>(joint_index)] =
        distribution(generator);
    }
  }

  moveit::core::RobotState state(model);
  std::ofstream output(argv[3]);
  output << std::setprecision(17);
  output << "{\n  \"gate\": \"UR-ANALYTIC-IK-FIT\",\n  \"scope\": \"MODEL_FIT_64_SAMPLES\",\n"
         << "  \"waypoint_survey\": false,\n  \"descartes\": false,\n"
         << "  \"source_commit\": \"b303c2948e715290ba7128f5739abbb4fef40546\",\n"
         << "  \"model\": \"UR5eParameters\",\n  \"extra_base_correction_applied\": false,\n"
         << "  \"rotation_error\": \"acos(clamp((trace(R_target^T R_fk)-1)/2, -1, 1))\",\n"
         << "  \"position_max_m\": " << kPositionMaxM << ",\n  \"orientation_max_rad\": " << kOrientationMaxRad
         << ",\n  \"diagnostic_min_sv\": " << kDiagnosticMinSv
         << ",\n  \"diagnostic_min_sv_role\": \"TAG_ONLY\",\n  \"sample_seed\": " << kSampleSeed
         << ",\n  \"sample_count\": " << kSampleCount << ",\n  \"root_link\": \"" << model->getRootLinkName()
         << "\",\n  \"joint_names\": [";
  for (int index = 0; index < 6; ++index)
  {
    if (index)
    {
      output << ", ";
    }
    output << "\"" << kJointNames[index] << "\"";
  }
  output << "],\n  \"limits_rad\": [";
  for (int index = 0; index < 6; ++index)
  {
    if (index)
    {
      output << ", ";
    }
    output << "[" << limits(index, 0) << ", " << limits(index, 1) << "]";
  }
  output << "],\n  \"samples\": [\n";

  int valid_samples = 0;
  int inconclusive_samples = 0;
  int fk_violation_samples = 0;
  int no_valid_samples = 0;
  int in_limit_fk_violations = 0;

  for (int sample_index = 0; sample_index < kSampleCount; ++sample_index)
  {
    const auto & sample = samples[static_cast<std::size_t>(sample_index)];
    std::vector<double> query(sample.begin(), sample.end());
    state.setJointGroupPositions(group, query);
    state.update();
    const Eigen::Isometry3d target = tool_in_base(state);
    Eigen::MatrixXd jacobian;
    state.getJacobian(
      group, state.getLinkModel("tool0"), Eigen::Vector3d::Zero(), jacobian, false);
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
    const double min_sv = svd.singularValues().minCoeff();
    const bool diagnostic_tag = min_sv < kDiagnosticMinSv;

    tesseract::common::LinkIdTransformMap poses;
    poses.emplace(tip_link, target);
    const Eigen::VectorXd seed = Eigen::VectorXd::Zero(6);
    tesseract::kinematics::IKSolutions solved;
    solver.calcInvKin(solved, poses, seed);

    struct Candidate
    {
      std::string source;
      Eigen::VectorXd joints;
      bool finite = false;
      bool in_limits = false;
      double position_error_m = std::numeric_limits<double>::quiet_NaN();
      double orientation_error_rad = std::numeric_limits<double>::quiet_NaN();
      bool fk_pass = false;
    };
    std::vector<Candidate> candidates;
    auto append_solution = [&](const Eigen::VectorXd & joints, const std::string & source) {
      Candidate candidate;
      candidate.source = source;
      candidate.joints = joints;
      candidate.finite = joints.allFinite();
      candidate.in_limits = candidate.finite && within_limits(joints, limits);
      if (candidate.finite)
      {
        std::vector<double> values(joints.data(), joints.data() + joints.size());
        state.setJointGroupPositions(group, values);
        state.update();
        const Eigen::Isometry3d actual = tool_in_base(state);
        candidate.position_error_m = (actual.translation() - target.translation()).norm();
        candidate.orientation_error_rad = rotation_error_rad(target.rotation(), actual.rotation());
        candidate.fk_pass = candidate.in_limits && candidate.position_error_m <= kPositionMaxM &&
                            candidate.orientation_error_rad <= kOrientationMaxRad;
      }
      candidates.push_back(std::move(candidate));
    };
    for (const auto & solution : solved)
    {
      append_solution(solution, "solver");
      if (!solution.allFinite())
      {
        continue;
      }
      const auto expanded = tesseract::kinematics::getRedundantSolutions<double>(solution, limits, redundant_joints);
      for (const auto & extra : expanded)
      {
        append_solution(extra, "redundant_2pi");
      }
    }

    int valid_count = 0;
    bool in_limit_fk_fail = false;
    for (const auto & candidate : candidates)
    {
      if (!candidate.in_limits)
      {
        continue;
      }
      if (candidate.fk_pass)
      {
        ++valid_count;
      }
      else
      {
        in_limit_fk_fail = true;
        ++in_limit_fk_violations;
      }
    }
    std::string status = "HAS_VALID_CANDIDATE";
    if (diagnostic_tag && valid_count == 0 && !in_limit_fk_fail)
    {
      status = "INCONCLUSIVE";
      ++inconclusive_samples;
    }
    else if (in_limit_fk_fail)
    {
      status = "CANDIDATE_SET_FK_VIOLATION";
      ++fk_violation_samples;
    }
    else if (valid_count == 0)
    {
      status = "NO_VALID_CANDIDATE";
      ++no_valid_samples;
    }
    else
    {
      ++valid_samples;
    }

    if (sample_index)
    {
      output << ",\n";
    }
    output << "    {\"index\": " << sample_index << ", \"joints_rad\": ";
    write_vector(output, query);
    output << ", \"min_singular_value\": ";
    write_number(output, min_sv);
    output << ", \"diagnostic_tag\": " << (diagnostic_tag ? "true" : "false")
           << ", \"solver_solution_count\": " << solved.size()
           << ", \"candidate_count\": " << candidates.size()
           << ", \"valid_count\": " << valid_count
           << ", \"status\": \"" << status << "\", \"candidates\": [";
    for (std::size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index)
    {
      const auto & candidate = candidates[candidate_index];
      if (candidate_index)
      {
        output << ", ";
      }
      std::vector<double> values(candidate.joints.data(), candidate.joints.data() + candidate.joints.size());
      output << "{\"source\": \"" << candidate.source << "\", \"joints_rad\": ";
      write_vector(output, values);
      output << ", \"finite\": " << (candidate.finite ? "true" : "false")
             << ", \"in_limits\": " << (candidate.in_limits ? "true" : "false")
             << ", \"position_error_m\": ";
      write_number(output, candidate.position_error_m);
      output << ", \"orientation_error_rad\": ";
      write_number(output, candidate.orientation_error_rad);
      output << ", \"fk_pass\": " << (candidate.fk_pass ? "true" : "false") << "}";
    }
    output << "]}";
    std::cout << "SAMPLE " << sample_index << " status " << status << " valid " << valid_count
              << " candidates " << candidates.size() << " tag " << (diagnostic_tag ? "YES" : "NO") << "\n"
              << std::flush;
  }

  const bool existence_pass = (valid_samples == kSampleCount);
  const bool candidate_set_pass = (in_limit_fk_violations == 0 && fk_violation_samples == 0);
  const bool model_pass = existence_pass && candidate_set_pass && inconclusive_samples == 0;
  output << "\n  ],\n  \"valid_samples\": " << valid_samples
         << ",\n  \"inconclusive_samples\": " << inconclusive_samples
         << ",\n  \"fk_violation_samples\": " << fk_violation_samples
         << ",\n  \"no_valid_samples\": " << no_valid_samples
         << ",\n  \"in_limit_fk_violations\": " << in_limit_fk_violations
         << ",\n  \"existence_pass\": " << (existence_pass ? "true" : "false")
         << ",\n  \"candidate_set_pass\": " << (candidate_set_pass ? "true" : "false")
         << ",\n  \"model_pass\": " << (model_pass ? "true" : "false") << "\n}\n";
  std::cout << "MODEL_PASS " << (model_pass ? "YES" : "NO") << " existence " << valid_samples
            << " inconclusive " << inconclusive_samples << " fk_violation_samples " << fk_violation_samples << "\n";
  rclcpp::shutdown();
  return model_pass ? 0 : 1;
}
