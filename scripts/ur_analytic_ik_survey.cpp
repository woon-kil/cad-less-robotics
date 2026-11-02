// UR-ANALYTIC-IK-WAYPOINT-SURVEY. 741개 pose의 후보 집합만 기록한다.
// 연속 경로 탐색과 Descartes는 호출하지 않는다.

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
const int kWaypointCount = 741;
const double kPositionMaxM = 1.0e-6;
const double kOrientationMaxRad = 1.0e-6;
const double kLimitBoundaryRad = 1.0e-12;
const double kDuplicateAbsRad = 1.0e-12;
const double kTwoPiClassAbsRad = 1.0e-8;

struct PoseRecord
{
  int sequence_id = -1;
  std::string segment_type;
  int process_run_id = -1;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
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
    if (!std::isfinite(joints[index]) ||
        joints[index] < limits(index, 0) - kLimitBoundaryRad ||
        joints[index] > limits(index, 1) + kLimitBoundaryRad)
    {
      return false;
    }
  }
  return true;
}

bool exact_same_joints(const Eigen::VectorXd & left, const Eigen::VectorXd & right)
{
  for (int index = 0; index < left.size(); ++index)
  {
    if (std::abs(left[index] - right[index]) > kDuplicateAbsRad)
    {
      return false;
    }
  }
  return true;
}

bool same_two_pi_class(const Eigen::VectorXd & left, const Eigen::VectorXd & right)
{
  for (int index = 0; index < left.size(); ++index)
  {
    const double delta = left[index] - right[index];
    const double turns = std::round(delta / (2.0 * M_PI));
    if (std::abs(delta - turns * 2.0 * M_PI) > kTwoPiClassAbsRad)
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

std::vector<PoseRecord> read_poses(const std::string & path)
{
  std::ifstream input(path);
  int count = 0;
  input >> count;
  std::vector<PoseRecord> poses;
  poses.reserve(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index)
  {
    PoseRecord pose;
    double x = 0, y = 0, z = 0, qx = 0, qy = 0, qz = 0, qw = 0;
    input >> pose.sequence_id >> pose.segment_type >> pose.process_run_id >> x >> y >> z >> qx >> qy >> qz >> qw;
    pose.position = Eigen::Vector3d(x, y, z);
    pose.orientation = Eigen::Quaterniond(qw, qx, qy, qz);
    poses.push_back(pose);
  }
  return poses;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 6)
  {
    std::cerr << "usage: ur_analytic_ik_survey urdf srdf poses survey_json search_json\n";
    return 2;
  }
  const auto poses = read_poses(argv[3]);
  if (static_cast<int>(poses.size()) != kWaypointCount)
  {
    std::cerr << "FAIL_WAYPOINT_COUNT " << poses.size() << "\n";
    return 2;
  }
  for (int index = 0; index < kWaypointCount; ++index)
  {
    const auto & pose = poses[static_cast<std::size_t>(index)];
    if (pose.sequence_id != index ||
        (pose.segment_type != "PROCESS" && pose.segment_type != "TRANSITION"))
    {
      std::cerr << "FAIL_POSE_IDENTITY " << index << "\n";
      return 2;
    }
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ur_analytic_ik_survey");
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
  Eigen::MatrixX2d limits(6, 2);
  for (int index = 0; index < 6; ++index)
  {
    const auto & bounds = group->getActiveJointModels()[static_cast<std::size_t>(index)]->getVariableBounds();
    if (bounds.size() != 1 || !bounds[0].position_bounded_)
    {
      std::cerr << "FAIL_JOINT_LIMIT\n";
      return 3;
    }
    limits(index, 0) = bounds[0].min_position_;
    limits(index, 1) = bounds[0].max_position_;
  }

  std::vector<tesseract::common::JointId> joint_ids;
  for (const char * name : kJointNames)
  {
    joint_ids.emplace_back(name);
  }
  const tesseract::common::LinkId tip_link("tool0");
  tesseract::kinematics::URInvKin solver(
    tesseract::kinematics::UR5eParameters, tesseract::common::LinkId("base_link"), tip_link, joint_ids);
  const std::vector<Eigen::Index> redundant_joints{0, 1, 2, 3, 4, 5};
  moveit::core::RobotState state(model);

  std::ofstream survey(argv[4]);
  std::ofstream search(argv[5]);
  survey << std::setprecision(17);
  search << std::setprecision(17);
  survey << "{\n  \"gate\": \"UR-ANALYTIC-IK-WAYPOINT-SURVEY\",\n"
         << "  \"revision\": \"R1\",\n"
         << "  \"scope\": \"CANDIDATE_SET_PER_WAYPOINT\",\n"
         << "  \"continuous_path\": false,\n  \"descartes\": false,\n"
         << "  \"quaternion_normalization\": \"COMPUTATIONAL_COPY_ONLY\",\n"
         << "  \"waypoint_count\": " << kWaypointCount << ",\n"
         << "  \"duplicate_rule\": \"KEEP_FIRST_IDENTICAL_JOINT_VECTOR\",\n"
         << "  \"two_pi_variants_kept\": true,\n  \"waypoints\": [\n";
  search << "{\n  \"gate\": \"UR-ANALYTIC-IK-WAYPOINT-SURVEY\",\n"
         << "  \"revision\": \"R1\",\n"
         << "  \"role\": \"SEARCH_CANDIDATE_SET\",\n  \"continuous_path\": false,\n"
         << "  \"rotation_input\": \"orientation_quaternion_normalized_xyzw\",\n  \"waypoints\": [\n";

  int has_valid = 0;
  int fk_violation_waypoints = 0;
  int no_valid = 0;
  int quaternion_not_usable = 0;
  long solver_total = 0;
  long expanded_total = 0;
  long valid_total = 0;
  long duplicate_total = 0;
  long violation_total = 0;
  double max_position = 0.0;
  double max_orientation = 0.0;
  int min_solver = std::numeric_limits<int>::max();
  int max_solver = 0;
  int min_expanded = std::numeric_limits<int>::max();
  int max_expanded = 0;
  int min_valid = std::numeric_limits<int>::max();
  int max_valid = 0;

  for (int index = 0; index < kWaypointCount; ++index)
  {
    const auto & pose = poses[static_cast<std::size_t>(index)];
    const Eigen::Quaterniond raw_orientation = pose.orientation;
    const bool components_finite = std::isfinite(raw_orientation.x()) && std::isfinite(raw_orientation.y()) &&
                                   std::isfinite(raw_orientation.z()) && std::isfinite(raw_orientation.w());
    const double quaternion_norm = raw_orientation.norm();
    const bool norm_finite = std::isfinite(quaternion_norm);
    const bool norm_positive = norm_finite && quaternion_norm > 0.0;
    const bool quaternion_usable = components_finite && norm_positive;
    const Eigen::Quaterniond unit_orientation = quaternion_usable ? raw_orientation.normalized() : Eigen::Quaterniond::Identity();
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.linear() = unit_orientation.toRotationMatrix();
    target.translation() = pose.position;
    tesseract::kinematics::IKSolutions solved;
    if (quaternion_usable)
    {
      tesseract::common::LinkIdTransformMap tip_poses;
      tip_poses.emplace(tip_link, target);
      const Eigen::VectorXd seed = Eigen::VectorXd::Zero(6);
      solver.calcInvKin(solved, tip_poses, seed);
    }

    struct Candidate
    {
      std::string source;
      Eigen::VectorXd joints;
      bool finite = false;
      bool duplicate = false;
      bool in_limits = false;
      int kinematic_class = -1;
      double position_error_m = std::numeric_limits<double>::quiet_NaN();
      double orientation_error_rad = std::numeric_limits<double>::quiet_NaN();
      bool fk_pass = false;
      bool valid = false;
      bool violation = false;
    };
    std::vector<Candidate> candidates;
    std::vector<Eigen::VectorXd> finite_joints;
    std::vector<int> finite_classes;
    int kinematic_class_count = 0;
    auto append_solution = [&](const Eigen::VectorXd & joints, const std::string & source) {
      Candidate candidate;
      candidate.source = source;
      candidate.joints = joints;
      candidate.finite = joints.allFinite();
      if (candidate.finite)
      {
        for (const auto & earlier : finite_joints)
        {
          if (exact_same_joints(joints, earlier))
          {
            candidate.duplicate = true;
            break;
          }
        }
        candidate.in_limits = within_limits(joints, limits);
        for (std::size_t class_index = 0; class_index < finite_joints.size(); ++class_index)
        {
          if (same_two_pi_class(joints, finite_joints[class_index]))
          {
            candidate.kinematic_class = finite_classes[class_index];
            break;
          }
        }
        if (candidate.kinematic_class < 0)
        {
          candidate.kinematic_class = kinematic_class_count;
          ++kinematic_class_count;
        }
        finite_joints.push_back(joints);
        finite_classes.push_back(candidate.kinematic_class);
        std::vector<double> values(joints.data(), joints.data() + joints.size());
        state.setJointGroupPositions(group, values);
        state.update();
        const Eigen::Isometry3d actual = tool_in_base(state);
        candidate.position_error_m = (actual.translation() - target.translation()).norm();
        candidate.orientation_error_rad = rotation_error_rad(target.rotation(), actual.rotation());
        candidate.fk_pass = candidate.in_limits && candidate.position_error_m <= kPositionMaxM &&
                            candidate.orientation_error_rad <= kOrientationMaxRad;
        if (candidate.in_limits)
        {
          max_position = std::max(max_position, candidate.position_error_m);
          max_orientation = std::max(max_orientation, candidate.orientation_error_rad);
        }
      }
      candidate.valid = candidate.finite && !candidate.duplicate && candidate.fk_pass;
      candidate.violation = candidate.finite && !candidate.duplicate && candidate.in_limits && !candidate.fk_pass;
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

    int solver_count = static_cast<int>(solved.size());
    int expanded_count = static_cast<int>(candidates.size());
    int valid_count = 0;
    int duplicate_count = 0;
    int nonfinite_count = 0;
    int out_of_limit_count = 0;
    int violation_count = 0;
    for (const auto & candidate : candidates)
    {
      if (!candidate.finite)
      {
        ++nonfinite_count;
      }
      else if (candidate.duplicate)
      {
        ++duplicate_count;
      }
      else if (!candidate.in_limits)
      {
        ++out_of_limit_count;
      }
      else if (candidate.violation)
      {
        ++violation_count;
      }
      else if (candidate.valid)
      {
        ++valid_count;
      }
    }
    if (nonfinite_count + duplicate_count + out_of_limit_count + violation_count + valid_count != expanded_count)
    {
      std::cerr << "FAIL_ACCOUNTING " << pose.sequence_id << "\n";
      return 6;
    }

    std::string status = "HAS_VALID_CANDIDATE";
    std::string reason = "";
    if (!quaternion_usable)
    {
      status = "QUATERNION_NOT_USABLE";
      reason = "QUATERNION_NOT_USABLE";
      ++quaternion_not_usable;
    }
    else if (violation_count > 0)
    {
      status = "CANDIDATE_SET_FK_VIOLATION";
      ++fk_violation_waypoints;
    }
    else if (valid_count == 0)
    {
      status = "NO_VALID_CANDIDATE";
      ++no_valid;
      if (solver_count == 0)
      {
        reason = "NO_SOLVER_SOLUTION";
      }
      else if (nonfinite_count == expanded_count)
      {
        reason = "ALL_NONFINITE";
      }
      else if (nonfinite_count == 0 && out_of_limit_count > 0 &&
               duplicate_count + out_of_limit_count == expanded_count)
      {
        reason = "ALL_OUT_OF_LIMITS";
      }
      else if (violation_count > 0)
      {
        reason = "FK_CRITERION";
      }
      else
      {
        reason = "UNCONFIRMED";
      }
    }
    else
    {
      ++has_valid;
    }
    solver_total += solver_count;
    expanded_total += expanded_count;
    valid_total += valid_count;
    duplicate_total += duplicate_count;
    violation_total += violation_count;
    min_solver = std::min(min_solver, solver_count);
    max_solver = std::max(max_solver, solver_count);
    min_expanded = std::min(min_expanded, expanded_count);
    max_expanded = std::max(max_expanded, expanded_count);
    min_valid = std::min(min_valid, valid_count);
    max_valid = std::max(max_valid, valid_count);

    if (index)
    {
      survey << ",\n";
      search << ",\n";
    }
    survey << "    {\"sequence_id\": " << pose.sequence_id
           << ", \"segment_type\": \"" << pose.segment_type << "\", \"process_run_id\": ";
    search << "    {\"sequence_id\": " << pose.sequence_id
           << ", \"segment_type\": \"" << pose.segment_type << "\", \"process_run_id\": ";
    if (pose.process_run_id < 0)
    {
      survey << "null";
      search << "null";
    }
    else
    {
      survey << pose.process_run_id;
      search << pose.process_run_id;
    }
    survey << ", \"command_position_m\": [";
    write_number(survey, pose.position.x());
    survey << ", ";
    write_number(survey, pose.position.y());
    survey << ", ";
    write_number(survey, pose.position.z());
    survey << "], \"orientation_quaternion_xyzw\": [";
    write_number(survey, raw_orientation.x());
    survey << ", ";
    write_number(survey, raw_orientation.y());
    survey << ", ";
    write_number(survey, raw_orientation.z());
    survey << ", ";
    write_number(survey, raw_orientation.w());
    survey << "], \"quaternion_norm\": ";
    write_number(survey, quaternion_norm);
    survey << ", \"quaternion_components_finite\": " << (components_finite ? "true" : "false")
           << ", \"quaternion_norm_finite\": " << (norm_finite ? "true" : "false")
           << ", \"quaternion_norm_positive\": " << (norm_positive ? "true" : "false")
           << ", \"orientation_quaternion_normalized_xyzw\": ";
    search << ", \"orientation_quaternion_xyzw\": [";
    write_number(search, raw_orientation.x());
    search << ", ";
    write_number(search, raw_orientation.y());
    search << ", ";
    write_number(search, raw_orientation.z());
    search << ", ";
    write_number(search, raw_orientation.w());
    search << "], \"quaternion_norm\": ";
    write_number(search, quaternion_norm);
    search << ", \"orientation_quaternion_normalized_xyzw\": ";
    if (quaternion_usable)
    {
      survey << "[";
      write_number(survey, unit_orientation.x());
      survey << ", ";
      write_number(survey, unit_orientation.y());
      survey << ", ";
      write_number(survey, unit_orientation.z());
      survey << ", ";
      write_number(survey, unit_orientation.w());
      survey << "]";
      search << "[";
      write_number(search, unit_orientation.x());
      search << ", ";
      write_number(search, unit_orientation.y());
      search << ", ";
      write_number(search, unit_orientation.z());
      search << ", ";
      write_number(search, unit_orientation.w());
      search << "]";
    }
    else
    {
      survey << "null";
      search << "null";
    }
    survey << ", \"solver_solution_count\": " << solver_count
           << ", \"expanded_candidate_count\": " << expanded_count
           << ", \"valid_count\": " << valid_count
           << ", \"exact_duplicate_count\": " << duplicate_count
           << ", \"nonfinite_count\": " << nonfinite_count
           << ", \"out_of_limit_count\": " << out_of_limit_count
           << ", \"in_limit_fk_violation_count\": " << violation_count
           << ", \"kinematic_class_count\": " << kinematic_class_count
           << ", \"status\": \"" << status << "\", \"reason\": ";
    if (reason.empty())
    {
      survey << "null";
    }
    else
    {
      survey << "\"" << reason << "\"";
    }
    survey << ", \"candidates\": [";
    search << ", \"candidates\": [";
    bool first_valid = true;
    for (std::size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index)
    {
      const auto & candidate = candidates[candidate_index];
      if (candidate_index)
      {
        survey << ", ";
      }
      std::vector<double> values(candidate.joints.data(), candidate.joints.data() + candidate.joints.size());
      survey << "{\"source\": \"" << candidate.source << "\", \"joints_rad\": ";
      write_vector(survey, values);
      survey << ", \"finite\": " << (candidate.finite ? "true" : "false")
             << ", \"exact_duplicate\": " << (candidate.duplicate ? "true" : "false")
             << ", \"in_limits\": " << (candidate.in_limits ? "true" : "false")
             << ", \"kinematic_class_id\": ";
      if (candidate.kinematic_class < 0)
      {
        survey << "null";
      }
      else
      {
        survey << candidate.kinematic_class;
      }
      survey << ", \"position_error_m\": ";
      write_number(survey, candidate.position_error_m);
      survey << ", \"orientation_error_rad\": ";
      write_number(survey, candidate.orientation_error_rad);
      survey << ", \"fk_pass\": " << (candidate.fk_pass ? "true" : "false")
             << ", \"valid\": " << (candidate.valid ? "true" : "false") << "}";
      if (candidate.valid)
      {
        if (!first_valid)
        {
          search << ", ";
        }
        first_valid = false;
        search << "{\"source\": \"" << candidate.source << "\", \"kinematic_class_id\": "
               << candidate.kinematic_class << ", \"joints_rad\": ";
        write_vector(search, values);
        search << "}";
      }
    }
    survey << "]}";
    search << "]}";
    std::cout << "WAYPOINT " << pose.sequence_id << " " << status << " solver " << solver_count
              << " expanded " << expanded_count << " valid " << valid_count << "\n"
              << std::flush;
  }

  survey << "\n  ],\n  \"has_valid_waypoints\": " << has_valid
         << ",\n  \"fk_violation_waypoints\": " << fk_violation_waypoints
         << ",\n  \"no_valid_waypoints\": " << no_valid
         << ",\n  \"quaternion_not_usable_waypoints\": " << quaternion_not_usable
         << ",\n  \"solver_solution_total\": " << solver_total
         << ",\n  \"expanded_candidate_total\": " << expanded_total
         << ",\n  \"valid_total\": " << valid_total
         << ",\n  \"exact_duplicate_total\": " << duplicate_total
         << ",\n  \"in_limit_fk_violation_total\": " << violation_total
         << ",\n  \"solver_solution_min\": " << min_solver
         << ",\n  \"solver_solution_max\": " << max_solver
         << ",\n  \"expanded_candidate_min\": " << min_expanded
         << ",\n  \"expanded_candidate_max\": " << max_expanded
         << ",\n  \"valid_min\": " << min_valid
         << ",\n  \"valid_max\": " << max_valid
         << ",\n  \"max_in_limit_position_error_m\": ";
  write_number(survey, max_position);
  survey << ",\n  \"max_in_limit_orientation_error_rad\": ";
  write_number(survey, max_orientation);
  survey << ",\n  \"continuous_path_claimed\": false\n}\n";
  search << "\n  ]\n}\n";
  std::cout << "SURVEY has_valid " << has_valid << " fk_violation " << fk_violation_waypoints
            << " no_valid " << no_valid << " solver " << solver_total
            << " expanded " << expanded_total << " valid " << valid_total << "\n";
  rclcpp::shutdown();
  return 0;
}
