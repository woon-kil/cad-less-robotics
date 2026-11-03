// 채택 관절 경로 하나에 MoveIt TOTG를 한 번 적용하고, 저장 메시지를
// PER_JOINT_QUINTIC_HERMITE로 검사한다. 호출 횟수는 1이다.

#include <openssl/sha.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometric_shapes/shapes.h>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <clocale>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
const char * kJointNames[] = {
  "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
const double kVelocityLimit = 3.141592653589793;
const double kAccelerationLimit = 5.0;
const double kPathTolerance = 0.1;
const double kResampleDt = 0.1;
const double kMinAngleChange = 0.001;
const double kPositionLimitM = 0.001;
const double kOrientationLimitRad = 0.01;
const double kLimitSlack = 1.0e-9;
const double kPositionOutside = 1.0e-12;
const double kEndpointRad = 1.0e-9;
const double kAbsentResidual = 0.1 + 1.0e-6;
const double kAmbiguousGap = 1.0e-9;
const double kInteriorU = 1.0e-6;
const double kRootImag = 1.0e-10;
const double kRootPlug = 1.0e-7;
const double kMinDt = 1.0e-12;
const int kSampleDenominator = 100;
const int kPointCap = 5000;
const double kPrepS = 30.0;
const double kGenerateS = 60.0;
const double kVerifyS = 180.0;
const double kSaveS = 30.0;
const double kOverallS = 300.0;
const double kDatum[3] = {0.3244333936553, -0.0499999998932, 0.11133404082435};
const char * kCouponId = "complex_surface_coupon";

const char * kUrdfSha = "a21bf5fb70b3a1745bf2e4816e0f43654539737ba7f2585939ec773d159778d8";
const char * kSrdfSha = "ee0d940b486bca52534d5b112c1020e0eaa542130c1301af8dc2fda28c757dbb";
const char * kRunsSha = "4d51b28f0851114b8d36036d0fb96bb1681bf03f89ac31a47bf7d3a18eecfb26";
const char * kTransitionsSha = "cc21e6083a9ecb0c740382ccf3f9b8a7e3e9bdaea64017389525cbc04cd821c4";
const char * kInsertsSha = "ff1f59d4275cdee2e41f5b8ef0da3182480204eb32d2372e1eb65ef4dd40f153";
const char * kMeshSha = "ea78058ca33866ef3c69764be48f90da4a6f2491455c9077c514ccecd6acbc16";
const char * kAcceptanceSha = "97fa8c989cfae92fc998003ceb010e4c585642fe13b157330a04cb162e610e44";
const char * kRestoreSha = "35f72af9e3b23b9cb2a4e6133eb24623489ed00762f8ccf40000e618a0998943";

struct PairName
{
  const char * first;
  const char * second;
};
const PairName kExpectedPairs[] = {
  {"base_link", "base_link_inertia"},
  {"base_link_inertia", "shoulder_link"},
  {"forearm_link", "upper_arm_link"},
  {"forearm_link", "wrist_1_link"},
  {"shoulder_link", "upper_arm_link"},
  {"tool0", "wrist_3_link"},
  {"wrist_1_link", "wrist_2_link"},
  {"wrist_2_link", "wrist_3_link"},
  {"tool0", "wrist_1_link"},
  {"tool0", "wrist_2_link"},
  {"wrist_1_link", "wrist_3_link"}};

struct MeshExpect
{
  const char * link;
  const char * uri;
  const char * sha;
};
const MeshExpect kMeshes[] = {
  {"base_link_inertia", "package://ur_description/meshes/ur5e/collision/base.stl",
    "9a145e0d46f2130afdf2a2e8825a00a929870c4c3d6d8e4d1adc5f04db3aac1b"},
  {"shoulder_link", "package://ur_description/meshes/ur5e/collision/shoulder.stl",
    "ceb92532177daa77682f5fbd628e01c2137d168f949a7a706ce1dabe9f002387"},
  {"upper_arm_link", "package://ur_description/meshes/ur5e/collision/upperarm.stl",
    "ee893044caf00075cb55b4cf666d1f1311c7979786212a501009f33bee945209"},
  {"forearm_link", "package://ur_description/meshes/ur5e/collision/forearm.stl",
    "2e7423cab807c34160ec4f770daee5e747d70e777eb01b7beeace2b8c5751816"},
  {"wrist_1_link", "package://ur_description/meshes/ur5e/collision/wrist1.stl",
    "f8c9f9337b6fd98c75f052e96de10e14a107ddb6874ba6b904e546f8a4e4f43a"},
  {"wrist_2_link", "package://ur_description/meshes/ur5e/collision/wrist2.stl",
    "2964a63f60ce3e3cf3ad55bcf190d7876d50e373cb64b70a57cea5885eaf3c86"},
  {"wrist_3_link", "package://ur_description/meshes/ur5e/collision/wrist3.stl",
    "83b3666b4ae2badd54af0d2c25a921682ecbc29e849eec646c3ed55fb74c78a3"}};

struct Pose
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::VectorXd joints = Eigen::VectorXd::Zero(6);
};

struct Chord
{
  bool transition = false;
  int original_from = -1;
  int original_to = -1;
  int half = 0;
};

struct Point6
{
  double time = 0.0;
  double position[6] = {};
  double velocity[6] = {};
  double acceleration[6] = {};
};

std::chrono::steady_clock::time_point g_started;
int g_totg_calls = 0;

double elapsed()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - g_started).count();
}

std::string sha256_file(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  SHA256_CTX context;
  SHA256_Init(&context);
  char buffer[1 << 16];
  while (input)
  {
    input.read(buffer, sizeof(buffer));
    if (input.gcount() > 0)
    {
      SHA256_Update(&context, buffer, static_cast<std::size_t>(input.gcount()));
    }
  }
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256_Final(digest, &context);
  std::ostringstream hex;
  hex << std::hex << std::setfill('0');
  for (unsigned char byte : digest)
  {
    hex << std::setw(2) << static_cast<int>(byte);
  }
  return hex.str();
}

std::string read_text(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

template <typename T>
bool read_pod(std::istream & input, T * value)
{
  input.read(reinterpret_cast<char *>(value), sizeof(T));
  return static_cast<bool>(input);
}

void write_num(std::ostream & output, double value)
{
  if (!std::isfinite(value))
  {
    output << "null";
    return;
  }
  output << std::setprecision(17) << value;
}

struct Quintic
{
  double duration = 0.0;
  double c[6][6] = {};
  bool finite = false;
};

double horner(const double * coeff, int degree, double tau)
{
  double value = coeff[degree];
  for (int index = degree - 1; index >= 0; --index)
  {
    value = value * tau + coeff[index];
  }
  return value;
}

Quintic make_quintic(double duration, const double position0[6], const double velocity0[6], const double acceleration0[6],
  const double position1[6], const double velocity1[6], const double acceleration1[6])
{
  Quintic quintic;
  quintic.duration = duration;
  quintic.finite = std::isfinite(duration) && duration > kMinDt;
  for (int joint = 0; joint < 6; ++joint)
  {
    const double values[] = {
      position0[joint], velocity0[joint], acceleration0[joint],
      position1[joint], velocity1[joint], acceleration1[joint]};
    for (double value : values)
    {
      if (!std::isfinite(value))
      {
        quintic.finite = false;
      }
    }
    const double v0 = velocity0[joint] * duration;
    const double a0 = acceleration0[joint] * duration * duration;
    const double v1 = velocity1[joint] * duration;
    const double a1 = acceleration1[joint] * duration * duration;
    const double p0 = position0[joint];
    const double p1 = position1[joint];
    double * coeff = quintic.c[joint];
    coeff[0] = p0;
    coeff[1] = v0;
    coeff[2] = 0.5 * a0;
    coeff[3] = -10.0 * p0 - 6.0 * v0 - 1.5 * a0 + 10.0 * p1 - 4.0 * v1 + 0.5 * a1;
    coeff[4] = 15.0 * p0 + 8.0 * v0 + 1.5 * a0 - 15.0 * p1 + 7.0 * v1 - a1;
    coeff[5] = -6.0 * p0 - 3.0 * v0 - 0.5 * a0 + 6.0 * p1 - 3.0 * v1 + 0.5 * a1;
  }
  return quintic;
}

double position_at(const Quintic & quintic, int joint, double tau)
{
  return horner(quintic.c[joint], 5, tau);
}

double d1(const double * coeff, double tau)
{
  double derived[5];
  for (int index = 0; index < 5; ++index)
  {
    derived[index] = (index + 1) * coeff[index + 1];
  }
  return horner(derived, 4, tau);
}

double d2(const double * coeff, double tau)
{
  double derived[4] = {2.0 * coeff[2], 6.0 * coeff[3], 12.0 * coeff[4], 20.0 * coeff[5]};
  return horner(derived, 3, tau);
}

double d3(const double * coeff, double tau)
{
  double derived[3] = {6.0 * coeff[3], 24.0 * coeff[4], 60.0 * coeff[5]};
  return horner(derived, 2, tau);
}

bool derivative_coeffs(const double * coeff, int order, double out[6], int * degree)
{
  double current[6];
  std::copy(coeff, coeff + 6, current);
  int deg = 5;
  for (int step = 0; step < order; ++step)
  {
    double next[6] = {};
    for (int index = 0; index < deg; ++index)
    {
      next[index] = (index + 1) * current[index + 1];
    }
    std::copy(next, next + 6, current);
    --deg;
  }
  std::copy(current, current + 6, out);
  *degree = deg;
  return true;
}

bool identically_small(const double * coeff, int degree)
{
  double scale = 0.0;
  for (int index = 0; index <= degree; ++index)
  {
    scale = std::max(scale, std::abs(coeff[index]));
  }
  return scale <= 1.0e-12;
}

bool polynomial_roots(const double * coeff, int degree, std::vector<double> * roots, bool * root_fail)
{
  roots->clear();
  while (degree > 0 && std::abs(coeff[degree]) <= 1.0e-14 * std::max(1.0, std::abs(coeff[0])))
  {
    --degree;
  }
  if (degree <= 0)
  {
    return true;
  }
  Eigen::MatrixXd companion = Eigen::MatrixXd::Zero(degree, degree);
  for (int row = 1; row < degree; ++row)
  {
    companion(row, row - 1) = 1.0;
  }
  for (int row = 0; row < degree; ++row)
  {
    companion(row, degree - 1) = -coeff[row] / coeff[degree];
  }
  Eigen::EigenSolver<Eigen::MatrixXd> solver(companion, false);
  if (solver.info() != Eigen::Success)
  {
    *root_fail = true;
    return false;
  }
  const auto values = solver.eigenvalues();
  for (int index = 0; index < values.size(); ++index)
  {
    if (std::abs(values[index].imag()) > kRootImag)
    {
      continue;
    }
    const double real = values[index].real();
    if (!std::isfinite(real) || real <= 0.0 || real >= 1.0)
    {
      continue;
    }
    const double plugged = horner(coeff, degree, real);
    if (!std::isfinite(plugged) || std::abs(plugged) > kRootPlug)
    {
      *root_fail = true;
      return false;
    }
    roots->push_back(real);
  }
  return true;
}

struct LimitEval
{
  bool finite = true;
  bool root_fail = false;
  bool position_fail = false;
  bool velocity_fail = false;
  bool acceleration_fail = false;
  double max_velocity = 0.0;
  double max_acceleration = 0.0;
};

void consider_limit(LimitEval * eval, const Quintic & quintic, int joint, double tau, const Eigen::MatrixX2d & limits)
{
  const double tau_velocity = d1(quintic.c[joint], tau);
  const double tau_acceleration = d2(quintic.c[joint], tau);
  const double velocity = tau_velocity / quintic.duration;
  const double acceleration = tau_acceleration / (quintic.duration * quintic.duration);
  const double position = position_at(quintic, joint, tau);
  if (!std::isfinite(position) || !std::isfinite(velocity) || !std::isfinite(acceleration))
  {
    eval->finite = false;
    return;
  }
  eval->max_velocity = std::max(eval->max_velocity, std::abs(velocity));
  eval->max_acceleration = std::max(eval->max_acceleration, std::abs(acceleration));
  if (position < limits(joint, 0) - kPositionOutside || position > limits(joint, 1) + kPositionOutside)
  {
    eval->position_fail = true;
  }
  if (std::abs(velocity) - kVelocityLimit > kLimitSlack)
  {
    eval->velocity_fail = true;
  }
  if (std::abs(acceleration) - kAccelerationLimit > kLimitSlack)
  {
    eval->acceleration_fail = true;
  }
}

LimitEval evaluate_limits(const Quintic & quintic, const Eigen::MatrixX2d & limits)
{
  LimitEval eval;
  if (!quintic.finite)
  {
    eval.finite = false;
    return eval;
  }
  for (int joint = 0; joint < 6; ++joint)
  {
    consider_limit(&eval, quintic, joint, 0.0, limits);
    consider_limit(&eval, quintic, joint, 1.0, limits);
    double coeff[6];
    int degree = 0;
    derivative_coeffs(quintic.c[joint], 1, coeff, &degree);
    std::vector<double> velocity_roots;
    if (!identically_small(coeff, degree) &&
        !polynomial_roots(coeff, degree, &velocity_roots, &eval.root_fail))
    {
      return eval;
    }
    derivative_coeffs(quintic.c[joint], 2, coeff, &degree);
    std::vector<double> acceleration_roots;
    if (!identically_small(coeff, degree) &&
        !polynomial_roots(coeff, degree, &acceleration_roots, &eval.root_fail))
    {
      return eval;
    }
    derivative_coeffs(quintic.c[joint], 3, coeff, &degree);
    std::vector<double> jerk_roots;
    if (!identically_small(coeff, degree) &&
        !polynomial_roots(coeff, degree, &jerk_roots, &eval.root_fail))
    {
      return eval;
    }
    for (double tau : velocity_roots)
    {
      consider_limit(&eval, quintic, joint, tau, limits);
    }
    for (double tau : acceleration_roots)
    {
      consider_limit(&eval, quintic, joint, tau, limits);
    }
    for (double tau : jerk_roots)
    {
      consider_limit(&eval, quintic, joint, tau, limits);
    }
  }
  return eval;
}

bool hermite_selftest(std::string * detail)
{
  Eigen::MatrixX2d limits(6, 2);
  limits.col(0).setConstant(-10.0);
  limits.col(1).setConstant(10.0);
  double p0[6] = {};
  double v0[6] = {};
  double a0[6] = {};
  double p1[6] = {};
  double v1[6] = {};
  double a1[6] = {};
  auto set_joint0 = [&](double position0, double velocity0, double acceleration0, double position1, double velocity1,
                       double acceleration1) {
    p0[0] = position0;
    v0[0] = velocity0;
    a0[0] = acceleration0;
    p1[0] = position1;
    v1[0] = velocity1;
    a1[0] = acceleration1;
  };
  // p(τ)=τ^2(1-τ)^2. 끝점 재현과 τ=0.5의 속도 0을 확인한다.
  set_joint0(0.0, 0.0, 2.0, 0.0, 0.0, 2.0);
  Quintic known = make_quintic(1.0, p0, v0, a0, p1, v1, a1);
  const double mid = position_at(known, 0, 0.5);
  const double mid_velocity = d1(known.c[0], 0.5);
  const double mid_acceleration = d2(known.c[0], 0.5);
  if (!known.finite || std::abs(mid - 0.0625) > 1.0e-12 || std::abs(mid_velocity) > 1.0e-12 ||
      std::abs(mid_acceleration + 1.0) > 1.0e-12)
  {
    *detail = "endpoint_or_midpoint_mismatch";
    return false;
  }
  LimitEval known_limits = evaluate_limits(known, limits);
  if (!known_limits.finite || known_limits.root_fail || known_limits.position_fail || known_limits.velocity_fail ||
      known_limits.acceleration_fail)
  {
    *detail = "known_polynomial_rejected";
    return false;
  }
  double velocity_coeff[6];
  int degree = 0;
  derivative_coeffs(known.c[0], 1, velocity_coeff, &degree);
  std::vector<double> roots;
  bool root_fail = false;
  if (!polynomial_roots(velocity_coeff, degree, &roots, &root_fail) || root_fail || roots.size() != 1 ||
      std::abs(roots[0] - 0.5) > 1.0e-8)
  {
    *detail = "interior_velocity_root";
    return false;
  }
  // jerk가 항등적으로 0인 2차식은 근 실패가 아니다.
  set_joint0(0.0, 0.0, 2.0, 1.0, 2.0, 2.0);
  Quintic quadratic = make_quintic(1.0, p0, v0, a0, p1, v1, a1);
  LimitEval quadratic_limits = evaluate_limits(quadratic, limits);
  if (!quadratic.finite || !quadratic_limits.finite || quadratic_limits.root_fail ||
      std::abs(position_at(quadratic, 0, 0.5) - 0.25) > 1.0e-12)
  {
    *detail = "zero_jerk_case";
    return false;
  }
  // 비유한 입력은 통과로 두지 않는다.
  set_joint0(0.0, std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0, 0.0, 0.0);
  Quintic nonfinite = make_quintic(1.0, p0, v0, a0, p1, v1, a1);
  LimitEval nonfinite_limits = evaluate_limits(nonfinite, limits);
  if (nonfinite.finite || nonfinite_limits.finite)
  {
    *detail = "nonfinite_accepted";
    return false;
  }
  // 끝 속도가 한계를 넘으면 실패다.
  limits.col(0).setConstant(-7.0);
  limits.col(1).setConstant(7.0);
  set_joint0(0.0, 10.0, 0.0, 0.1, 0.0, 0.0);
  Quintic fast = make_quintic(1.0, p0, v0, a0, p1, v1, a1);
  LimitEval fast_limits = evaluate_limits(fast, limits);
  if (!fast_limits.finite || !fast_limits.velocity_fail)
  {
    *detail = "velocity_violation_not_detected";
    return false;
  }
  *detail = "PASS";
  return true;
}

struct Waypoint
{
  int sequence_id = -1;
  Pose pose;
  std::vector<Eigen::VectorXd> candidates;
};

struct Insert
{
  int from_sequence_id = -1;
  int to_sequence_id = -1;
  Pose pose;
  std::vector<Eigen::VectorXd> candidates;
};

bool read_joints(std::istream & input, Eigen::VectorXd * joints)
{
  double values[6];
  input.read(reinterpret_cast<char *>(values), sizeof(values));
  *joints = Eigen::VectorXd(6);
  for (int index = 0; index < 6; ++index)
  {
    if (!std::isfinite(values[index]))
    {
      return false;
    }
    (*joints)[index] = values[index];
  }
  return static_cast<bool>(input);
}

bool load_runs(const std::string & path, std::vector<std::vector<Waypoint>> * runs)
{
  std::ifstream input(path, std::ios::binary);
  char magic[5] = {};
  input.read(magic, 5);
  std::uint32_t run_count = 0;
  if (!input || std::string(magic, 5) != "URPC1" || !read_pod(input, &run_count) || run_count != 21)
  {
    return false;
  }
  runs->resize(21);
  int count = 0;
  for (int run_id = 0; run_id < 21; ++run_id)
  {
    std::int32_t stored = 0;
    std::uint32_t waypoint_count = 0;
    if (!read_pod(input, &stored) || !read_pod(input, &waypoint_count) || stored != run_id)
    {
      return false;
    }
    (*runs)[static_cast<std::size_t>(run_id)].resize(waypoint_count);
    for (std::uint32_t index = 0; index < waypoint_count; ++index)
    {
      Waypoint & waypoint = (*runs)[static_cast<std::size_t>(run_id)][index];
      std::int32_t sequence = 0;
      if (!read_pod(input, &sequence))
      {
        return false;
      }
      waypoint.sequence_id = sequence;
      double position[3];
      double orientation[4];
      input.read(reinterpret_cast<char *>(position), sizeof(position));
      input.read(reinterpret_cast<char *>(orientation), sizeof(orientation));
      waypoint.pose.position = Eigen::Vector3d(position[0], position[1], position[2]);
      waypoint.pose.orientation = Eigen::Quaterniond(orientation[3], orientation[0], orientation[1], orientation[2]);
      std::uint32_t candidate_count = 0;
      if (!read_pod(input, &candidate_count) || candidate_count != 256)
      {
        return false;
      }
      waypoint.candidates.resize(256);
      for (Eigen::VectorXd & candidate : waypoint.candidates)
      {
        std::int32_t class_id = 0;
        std::uint8_t source = 0;
        if (!read_pod(input, &class_id) || !read_pod(input, &source) || (source != 0 && source != 1) ||
            !read_joints(input, &candidate))
        {
          return false;
        }
      }
      if (!waypoint.pose.position.allFinite() || !(waypoint.pose.orientation.norm() > 0.0))
      {
        return false;
      }
      waypoint.pose.orientation.normalize();
      ++count;
    }
  }
  return count == 721 && input && input.peek() == EOF;
}

bool load_transitions(const std::string & path, std::map<int, Waypoint> * by_id)
{
  const int expected[20] = {
    35, 70, 105, 140, 175, 210, 245, 281, 317, 353, 389, 425, 461, 496, 531, 566, 601, 636, 671, 706};
  std::ifstream input(path, std::ios::binary);
  char magic[8] = {};
  input.read(magic, 8);
  std::uint32_t count = 0;
  if (!input || std::string(magic, 4) != "TRN1" || !read_pod(input, &count) || count != 20)
  {
    return false;
  }
  for (int index = 0; index < 20; ++index)
  {
    std::int32_t stored = 0;
    std::int32_t from_run = 0;
    std::int32_t to_run = 0;
    std::int32_t sequence = 0;
    if (!read_pod(input, &stored) || !read_pod(input, &from_run) || !read_pod(input, &to_run) ||
        !read_pod(input, &sequence) || stored != index || sequence != expected[index])
    {
      return false;
    }
    double ignored[12];
    double position[3];
    double orientation[4];
    input.read(reinterpret_cast<char *>(ignored), sizeof(ignored));
    input.read(reinterpret_cast<char *>(position), sizeof(position));
    input.read(reinterpret_cast<char *>(orientation), sizeof(orientation));
    std::uint32_t candidate_count = 0;
    if (!read_pod(input, &candidate_count) || candidate_count != 256)
    {
      return false;
    }
    Waypoint waypoint;
    waypoint.sequence_id = sequence;
    waypoint.pose.position = Eigen::Vector3d(position[0], position[1], position[2]);
    waypoint.pose.orientation = Eigen::Quaterniond(orientation[3], orientation[0], orientation[1], orientation[2]);
    waypoint.candidates.resize(256);
    for (Eigen::VectorXd & candidate : waypoint.candidates)
    {
      std::uint8_t source = 0;
      std::int32_t class_id = 0;
      if (!read_pod(input, &source) || !read_pod(input, &class_id) || (source != 0 && source != 1) ||
          !read_joints(input, &candidate))
      {
        return false;
      }
    }
    if (!waypoint.pose.position.allFinite() || !(waypoint.pose.orientation.norm() > 0.0))
    {
      return false;
    }
    waypoint.pose.orientation.normalize();
    (*by_id)[sequence] = std::move(waypoint);
  }
  return static_cast<bool>(input) && input.peek() == EOF && by_id->size() == 20;
}

bool insert_span(int from_id, int to_id)
{
  const int spans[6][3] = {{6, 211, 244}, {7, 246, 280}, {8, 282, 316}, {9, 318, 352}, {10, 354, 388}, {11, 390, 424}};
  if (to_id != from_id + 1)
  {
    return false;
  }
  for (const auto & span : spans)
  {
    if (from_id >= span[1] && to_id <= span[2])
    {
      return true;
    }
  }
  return false;
}

bool load_inserts(const std::string & path, std::map<int, Insert> * inserts)
{
  const int run_ids[6] = {6, 7, 8, 9, 10, 11};
  const int counts[6] = {33, 34, 34, 34, 34, 34};
  std::ifstream input(path, std::ios::binary);
  char magic[4] = {};
  input.read(magic, 4);
  std::uint32_t run_count = 0;
  if (!input || std::string(magic, 4) != "R6IN" || !read_pod(input, &run_count) || run_count != 6)
  {
    return false;
  }
  for (int index = 0; index < 6; ++index)
  {
    std::int32_t run_id = 0;
    std::int32_t insert_count = 0;
    if (!read_pod(input, &run_id) || !read_pod(input, &insert_count) || run_id != run_ids[index] ||
        insert_count != counts[index])
    {
      return false;
    }
    for (int insert_index = 0; insert_index < insert_count; ++insert_index)
    {
      std::int32_t from_id = 0;
      std::int32_t to_id = 0;
      if (!read_pod(input, &from_id) || !read_pod(input, &to_id) || !insert_span(from_id, to_id))
      {
        return false;
      }
      double position[3];
      double orientation[4];
      input.read(reinterpret_cast<char *>(position), sizeof(position));
      input.read(reinterpret_cast<char *>(orientation), sizeof(orientation));
      std::uint32_t valid_count = 0;
      if (!read_pod(input, &valid_count) || valid_count != 256)
      {
        return false;
      }
      Insert insert;
      insert.from_sequence_id = from_id;
      insert.to_sequence_id = to_id;
      insert.pose.position = Eigen::Vector3d(position[0], position[1], position[2]);
      insert.pose.orientation = Eigen::Quaterniond(orientation[3], orientation[0], orientation[1], orientation[2]);
      insert.candidates.resize(256);
      for (Eigen::VectorXd & candidate : insert.candidates)
      {
        std::int32_t class_id = 0;
        std::uint8_t source = 0;
        if (!read_pod(input, &class_id) || !read_pod(input, &source) || (source != 0 && source != 1) ||
            !read_joints(input, &candidate))
        {
          return false;
        }
      }
      if (!insert.pose.position.allFinite() || !(insert.pose.orientation.norm() > 0.0))
      {
        return false;
      }
      insert.pose.orientation.normalize();
      (*inserts)[from_id] = std::move(insert);
    }
  }
  return static_cast<bool>(input) && input.peek() == EOF && inserts->size() == 203;
}

bool assemble(
  const std::vector<std::vector<Waypoint>> & runs,
  const std::map<int, Waypoint> & transitions,
  const std::map<int, Insert> & inserts,
  std::vector<Pose> * poses,
  std::vector<Chord> * chords)
{
  const int transition_ids[20] = {
    35, 70, 105, 140, 175, 210, 245, 281, 317, 353, 389, 425, 461, 496, 531, 566, 601, 636, 671, 706};
  std::map<int, Waypoint> by_id = transitions;
  for (const auto & run : runs)
  {
    for (const Waypoint & waypoint : run)
    {
      if (!by_id.emplace(waypoint.sequence_id, waypoint).second)
      {
        return false;
      }
    }
  }
  if (by_id.size() != 741)
  {
    return false;
  }
  auto push_original = [&](int sequence) {
    const Waypoint & waypoint = by_id.at(sequence);
    Pose pose = waypoint.pose;
    pose.joints = waypoint.candidates.at(224);
    poses->push_back(pose);
  };
  push_original(0);
  int process_edges = 0;
  int transition_edges = 0;
  for (int sequence = 1; sequence < 741; ++sequence)
  {
    const bool transition = std::find(std::begin(transition_ids), std::end(transition_ids), sequence) !=
      std::end(transition_ids);
    const auto insert = inserts.find(sequence - 1);
    if (insert != inserts.end())
    {
      if (transition || insert->second.to_sequence_id != sequence)
      {
        return false;
      }
      Pose mid = insert->second.pose;
      mid.joints = insert->second.candidates.at(224);
      poses->push_back(mid);
      chords->push_back(Chord{false, sequence - 1, sequence, 1});
      push_original(sequence);
      chords->push_back(Chord{false, sequence - 1, sequence, 2});
      process_edges += 2;
    }
    else
    {
      push_original(sequence);
      chords->push_back(Chord{transition, sequence - 1, sequence, 0});
      if (transition)
      {
        ++transition_edges;
      }
      else
      {
        ++process_edges;
      }
    }
  }
  return poses->size() == 944 && chords->size() == 943 && process_edges == 923 && transition_edges == 20;
}

bool parse_restore_joints(const std::string & text, std::vector<Eigen::VectorXd> * joints)
{
  const std::string key = "\"joints_rad\": [";
  std::size_t cursor = 0;
  while (cursor < text.size())
  {
    const std::size_t found = text.find(key, cursor);
    if (found == std::string::npos)
    {
      break;
    }
    cursor = found + key.size();
    Eigen::VectorXd values(6);
    for (int index = 0; index < 6; ++index)
    {
      while (cursor < text.size() && (text[cursor] == ',' || text[cursor] == ' ' || text[cursor] == '\n' || text[cursor] == '\r' || text[cursor] == '\t'))
      {
        ++cursor;
      }
      char * end = nullptr;
      values[index] = std::strtod(text.c_str() + cursor, &end);
      if (end == text.c_str() + cursor || !std::isfinite(values[index]))
      {
        if (joints->empty())
        {
          std::cerr << "restore parse fail at " << cursor << " snippet " << text.substr(cursor, 40) << "\n";
        }
        return false;
      }
      cursor = static_cast<std::size_t>(end - text.c_str());
    }
    joints->push_back(values);
  }
  return joints->size() == 944;
}

double max_abs_diff(const Eigen::VectorXd & left, const Eigen::VectorXd & right)
{
  double maximum = 0.0;
  for (int index = 0; index < 6; ++index)
  {
    maximum = std::max(maximum, std::abs(left[index] - right[index]));
  }
  return maximum;
}

std::string resolve_package(const std::string & uri)
{
  const std::string prefix = "package://";
  const std::size_t slash = uri.find('/', prefix.size());
  if (uri.compare(0, prefix.size(), prefix) != 0 || slash == std::string::npos)
  {
    return "";
  }
  try
  {
    const std::string package = uri.substr(prefix.size(), slash - prefix.size());
    return ament_index_cpp::get_package_share_directory(package) + "/" + uri.substr(slash + 1);
  }
  catch (const std::exception &)
  {
    return "";
  }
}

bool meshes_ok(const urdf::ModelInterface & urdf, std::string * error)
{
  for (const MeshExpect & expected : kMeshes)
  {
    const auto link = urdf.getLink(expected.link);
    if (!link || link->collision_array.size() != 1 || !link->collision_array[0]->geometry)
    {
      *error = "robot_mesh_missing";
      return false;
    }
    const auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->collision_array[0]->geometry);
    if (!mesh || mesh->filename != expected.uri)
    {
      *error = "robot_mesh_uri";
      return false;
    }
    if (sha256_file(resolve_package(mesh->filename)) != expected.sha)
    {
      *error = std::string("robot_mesh_hash ") + expected.link;
      return false;
    }
  }
  return true;
}

bool load_coupon(const std::string & path, std::vector<double> * vertices, std::vector<std::uint32_t> * triangles)
{
  std::ifstream input(path, std::ios::binary);
  char magic[8] = {};
  input.read(magic, 8);
  std::uint32_t accepted = 0;
  std::uint32_t vertex_count = 0;
  std::uint32_t triangle_count = 0;
  if (!input || std::string(magic, 5) != "COUP1" || !read_pod(input, &accepted) || !read_pod(input, &vertex_count) ||
      !read_pod(input, &triangle_count) || accepted != 1 || vertex_count != 413 || triangle_count != 564)
  {
    return false;
  }
  vertices->resize(static_cast<std::size_t>(vertex_count) * 3);
  triangles->resize(static_cast<std::size_t>(triangle_count) * 3);
  input.read(reinterpret_cast<char *>(vertices->data()), vertices->size() * sizeof(double));
  input.read(reinterpret_cast<char *>(triangles->data()), triangles->size() * sizeof(std::uint32_t));
  return static_cast<bool>(input) && input.peek() == EOF;
}

bool scene_ok(const planning_scene::PlanningScene & scene, const std::vector<double> & vertices,
  const std::vector<std::uint32_t> & triangles)
{
  const auto object = scene.getWorld()->getObject(kCouponId);
  const shapes::Mesh * loaded = object && object->shapes_.size() == 1 ?
    dynamic_cast<const shapes::Mesh *>(object->shapes_[0].get()) : nullptr;
  if (!loaded || loaded->vertex_count != 413 || loaded->triangle_count != 564)
  {
    return false;
  }
  for (std::size_t index = 0; index < vertices.size(); ++index)
  {
    if (loaded->vertices[index] != vertices[index])
    {
      return false;
    }
  }
  if (object->pose_.rotation() != Eigen::Matrix3d::Identity() ||
      object->pose_.translation().x() != kDatum[0] || object->pose_.translation().y() != kDatum[1] ||
      object->pose_.translation().z() != kDatum[2])
  {
    return false;
  }
  const auto & matrix = scene.getAllowedCollisionMatrix();
  collision_detection::AllowedCollision::Type type;
  int always_pairs = 0;
  for (const PairName & pair : kExpectedPairs)
  {
    const std::string left = pair.first;
    const std::string right = pair.second;
    if (matrix.getEntry(left, right, type) && type == collision_detection::AllowedCollision::ALWAYS)
    {
      ++always_pairs;
    }
  }
  return always_pairs == 11 && scene.getWorld()->size() == 1;
}

struct Assignment
{
  enum Type
  {
    ABSENT,
    AMBIGUOUS,
    VERTEX,
    CHORD
  } type = ABSENT;
  int index = -1;
  double u = 0.0;
  double residual = 0.0;
  double left = 0.0;
};

Assignment assign_sample(const Eigen::VectorXd & sample, const std::vector<Pose> & poses)
{
  Assignment best;
  best.residual = std::numeric_limits<double>::infinity();
  double second = std::numeric_limits<double>::infinity();
  int second_index = -1;
  double second_u = 0.0;
  for (int chord = 0; chord < 943; ++chord)
  {
    const Eigen::VectorXd delta = poses[chord + 1].joints - poses[chord].joints;
    const double length2 = delta.squaredNorm();
    double u = 0.0;
    Eigen::VectorXd point = poses[chord].joints;
    if (length2 > 1.0e-24)
    {
      u = std::clamp((sample - poses[chord].joints).dot(delta) / length2, 0.0, 1.0);
      point = poses[chord].joints + u * delta;
    }
    const double residual = (sample - point).norm();
    if (residual < best.residual)
    {
      second = best.residual;
      second_index = best.index;
      second_u = best.u;
      best.residual = residual;
      best.index = chord;
      best.u = u;
      best.type = Assignment::CHORD;
    }
    else if (residual < second)
    {
      second = residual;
      second_index = chord;
      second_u = u;
    }
  }
  if (best.residual > kAbsentResidual)
  {
    best.type = Assignment::ABSENT;
    return best;
  }
  if (second - best.residual < kAmbiguousGap)
  {
    const int low = std::min(best.index, second_index);
    const int high = std::max(best.index, second_index);
    const bool adjacent = high == low + 1;
    const int vertex = high;
    double vertex_gap = std::numeric_limits<double>::infinity();
    if (adjacent)
    {
      vertex_gap = max_abs_diff(sample, poses[static_cast<std::size_t>(vertex)].joints);
    }
    if (adjacent && vertex_gap <= kEndpointRad)
    {
      best.type = Assignment::VERTEX;
      best.index = vertex;
      best.u = vertex == 0 ? 0.0 : 1.0;
      best.left = vertex == 0 ? -1.0 : static_cast<double>(vertex - 1);
      return best;
    }
    // 양 끝이 같은 꼭짓점에 붙는 경우도 공유 꼭짓점이다.
    for (int vertex_index = 0; vertex_index < 944; ++vertex_index)
    {
      if (max_abs_diff(sample, poses[static_cast<std::size_t>(vertex_index)].joints) <= kEndpointRad)
      {
        best.type = Assignment::VERTEX;
        best.index = vertex_index;
        best.left = vertex_index == 0 ? -1.0 : static_cast<double>(vertex_index - 1);
        return best;
      }
    }
    best.type = Assignment::AMBIGUOUS;
    (void)second_u;
    return best;
  }
  best.type = Assignment::CHORD;
  best.left = static_cast<double>(best.index);
  return best;
}

double rotation_error(const Eigen::Matrix3d & target, const Eigen::Matrix3d & actual)
{
  double cosine = ((target.transpose() * actual).trace() - 1.0) * 0.5;
  cosine = std::clamp(cosine, -1.0, 1.0);
  return std::acos(cosine);
}

struct FailureRecord
{
  int interval = -1;
  int sample = -1;
  std::string reason;
  double residual = 0.0;
};

struct CrossRecord
{
  int interval = -1;
  int from_chord = -1;
  int to_chord = -1;
  int between_k = -1;
};

void write_setup(const std::string & path, int continuous, const std::string & frame, const std::string & status)
{
  std::ofstream output(path);
  output << std::setprecision(17);
  output << "{\n"
         << "  \"algorithm\": \"trajectory_processing::TimeOptimalTrajectoryGeneration\",\n"
         << "  \"overload\": \"computeTimeStamps(trajectory, velocity_limits, acceleration_limits, 1.0, 1.0)\",\n"
         << "  \"path_tolerance_rad\": 0.1,\n"
         << "  \"resample_dt_s\": 0.1,\n"
         << "  \"min_angle_change_rad\": 0.001,\n"
         << "  \"velocity_scaling\": 1.0,\n"
         << "  \"acceleration_scaling\": 1.0,\n"
         << "  \"velocity_limit_rad_s\": " << kVelocityLimit << ",\n"
         << "  \"velocity_limit_class\": \"URDF_LIMIT\",\n"
         << "  \"acceleration_limit_rad_s2\": " << kAccelerationLimit << ",\n"
         << "  \"acceleration_limit_class\": \"MOVEIT_CONFIGURATION\",\n"
         << "  \"interpolation\": \"PER_JOINT_QUINTIC_HERMITE\",\n"
         << "  \"controller_interpolation\": \"NOT_CLAIMED\",\n"
         << "  \"continuous_joint_count\": " << continuous << ",\n"
         << "  \"compute_time_stamps_calls\": " << g_totg_calls << ",\n"
         << "  \"frame_id\": \"" << frame << "\",\n"
         << "  \"joint_names\": [\"shoulder_pan_joint\", \"shoulder_lift_joint\", \"elbow_joint\", \"wrist_1_joint\", \"wrist_2_joint\", \"wrist_3_joint\"],\n"
         << "  \"units\": \"rad\",\n"
         << "  \"tcp\": \"tool0\",\n"
         << "  \"pose_frame\": \"base_link\",\n"
         << "  \"restore_sha256\": \"" << kRestoreSha << "\",\n"
         << "  \"urdf_sha256\": \"" << kUrdfSha << "\",\n"
         << "  \"source_tag\": \"moveit2-2.12.4\",\n"
         << "  \"status\": \"" << status << "\"\n}\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  g_started = std::chrono::steady_clock::now();
  std::setlocale(LC_ALL, "C");
  if (argc != 10)
  {
    std::cerr << "EXECUTION_ERROR arguments\n";
    return 4;
  }
  const std::string urdf_path = argv[1];
  const std::string srdf_path = argv[2];
  const std::string runs_path = argv[3];
  const std::string transitions_path = argv[4];
  const std::string inserts_path = argv[5];
  const std::string mesh_path = argv[6];
  const std::string acceptance_path = argv[7];
  const std::string restore_path = argv[8];
  const std::string out_dir = argv[9];
  std::ofstream log(out_dir + "/parent.log", std::ios::app);
  auto fail = [&](const std::string & status, const std::string & phase) {
    write_setup(out_dir + "/setup.json", -1, "", status);
    std::ofstream checks(out_dir + "/checks.json");
    checks << "{\"totg_message_check\": \"FAIL\", \"phase\": \"" << phase << "\", \"status\": \"" << status
           << "\", \"compute_time_stamps_calls\": " << g_totg_calls
           << ", \"controller_interpolation\": \"NOT_CLAIMED\"}\n";
    std::cerr << status << " " << phase << "\n";
    return status == "HERMITE_SELFTEST_FAIL" ? 5 : 2;
  };
  std::string hermite_detail;
  if (!hermite_selftest(&hermite_detail))
  {
    std::ofstream hermite(out_dir + "/hermite_selftest.json");
    hermite << "{\"status\": \"FAIL\", \"detail\": \"" << hermite_detail << "\"}\n";
    return fail("HERMITE_SELFTEST_FAIL", "PREP");
  }
  {
    std::ofstream hermite(out_dir + "/hermite_selftest.json");
    hermite << "{\"status\": \"PASS\", \"detail\": \"known quintic midpoint, interior velocity root, zero jerk, nonfinite rejected, velocity violation detected\"}\n";
  }
  if (sha256_file(urdf_path) != kUrdfSha || sha256_file(srdf_path) != kSrdfSha || sha256_file(runs_path) != kRunsSha ||
      sha256_file(transitions_path) != kTransitionsSha || sha256_file(inserts_path) != kInsertsSha ||
      sha256_file(mesh_path) != kMeshSha || sha256_file(acceptance_path) != kAcceptanceSha ||
      sha256_file(restore_path) != kRestoreSha)
  {
    return fail("BLOCKED_INPUT_INTEGRITY", "PREP");
  }
  if (read_text(acceptance_path).find("\"accepted\": true") == std::string::npos)
  {
    return fail("BLOCKED_INPUT_INTEGRITY", "ACCEPTANCE");
  }
  std::vector<std::vector<Waypoint>> runs;
  std::map<int, Waypoint> transitions;
  std::map<int, Insert> inserts;
  std::vector<Pose> poses;
  std::vector<Chord> chords;
  std::vector<Eigen::VectorXd> restore_joints;
  const bool runs_ok = load_runs(runs_path, &runs);
  const bool transitions_ok = runs_ok && load_transitions(transitions_path, &transitions);
  const bool inserts_ok = transitions_ok && load_inserts(inserts_path, &inserts);
  const bool assemble_ok = inserts_ok && assemble(runs, transitions, inserts, &poses, &chords);
  const bool restore_ok = assemble_ok && parse_restore_joints(read_text(restore_path), &restore_joints);
  std::cerr << "load runs " << runs_ok << " transitions " << transitions_ok << " inserts " << inserts_ok
            << " assemble " << assemble_ok << " poses " << poses.size() << " chords " << chords.size()
            << " restore " << restore_ok << " restore_n " << restore_joints.size() << "\n";
  if (!restore_ok)
  {
    return fail("INPUT_MAPPING_FAIL", "PREP");
  }
  for (int index = 0; index < 944; ++index)
  {
    if (max_abs_diff(restore_joints[static_cast<std::size_t>(index)], poses[static_cast<std::size_t>(index)].joints) > 0.0)
    {
      return fail("INPUT_MAPPING_FAIL", "RESTORE_JOINTS");
    }
  }
  std::vector<double> vertices;
  std::vector<std::uint32_t> triangles;
  if (!load_coupon(mesh_path, &vertices, &triangles))
  {
    return fail("BLOCKED_INPUT_INTEGRITY", "MESH");
  }
  rclcpp::init(argc, argv);
  auto urdf = urdf::parseURDF(read_text(urdf_path));
  auto srdf = std::make_shared<srdf::Model>();
  std::string mesh_error;
  if (!urdf || !srdf->initString(*urdf, read_text(srdf_path)) || !meshes_ok(*urdf, &mesh_error))
  {
    rclcpp::shutdown();
    return fail("BLOCKED_INPUT_INTEGRITY", mesh_error.empty() ? "ROBOT" : mesh_error);
  }
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  const auto * group = model->getJointModelGroup("ur_manipulator");
  if (group == nullptr || group->getActiveJointModelNames().size() != 6 ||
      !group->getContinuousJointModels().empty())
  {
    write_setup(out_dir + "/setup.json", group ? static_cast<int>(group->getContinuousJointModels().size()) : -1, "",
      "CONTINUOUS_JOINT_PRESENT");
    rclcpp::shutdown();
    return fail("CONTINUOUS_JOINT_PRESENT", "PREP");
  }
  for (int index = 0; index < 6; ++index)
  {
    if (group->getActiveJointModelNames()[static_cast<std::size_t>(index)] != kJointNames[index])
    {
      rclcpp::shutdown();
      return fail("BLOCKED_INPUT_INTEGRITY", "JOINT_ORDER");
    }
  }
  Eigen::MatrixX2d limits(6, 2);
  for (int index = 0; index < 6; ++index)
  {
    const auto & bounds = group->getActiveJointModels()[static_cast<std::size_t>(index)]->getVariableBounds();
    if (bounds.size() != 1 || !bounds[0].position_bounded_)
    {
      rclcpp::shutdown();
      return fail("BLOCKED_INPUT_INTEGRITY", "POSITION_LIMITS");
    }
    limits(index, 0) = bounds[0].min_position_;
    limits(index, 1) = bounds[0].max_position_;
  }
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  auto raw = std::make_shared<shapes::Mesh>(413, 564);
  std::copy(vertices.begin(), vertices.end(), raw->vertices);
  for (std::size_t index = 0; index < triangles.size(); ++index)
  {
    raw->triangles[index] = triangles[index];
  }
  Eigen::Isometry3d object_pose = Eigen::Isometry3d::Identity();
  object_pose.translation() << kDatum[0], kDatum[1], kDatum[2];
  scene->getWorldNonConst()->addToObject(kCouponId, object_pose, raw, Eigen::Isometry3d::Identity());
  if (!scene_ok(*scene, vertices, triangles))
  {
    rclcpp::shutdown();
    return fail("BLOCKED_INPUT_INTEGRITY", "SCENE");
  }
  collision_detection::AllowedCollisionMatrix acm = planning_scene::PlanningScene(model).getAllowedCollisionMatrix();
  collision_detection::CollisionRequest request;
  request.group_name = "ur_manipulator";
  request.pad_environment_collisions = true;
  request.pad_self_collisions = false;
  request.contacts = true;
  request.max_contacts = 20;
  request.max_contacts_per_pair = 4;
  request.distance = false;
  if (elapsed() > kPrepS)
  {
    rclcpp::shutdown();
    return fail("PREP_CAP", "PREP");
  }
  robot_trajectory::RobotTrajectory trajectory(model, "ur_manipulator");
  for (const Pose & pose : poses)
  {
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    std::vector<double> values(pose.joints.data(), pose.joints.data() + 6);
    state.setJointGroupPositions(group, values);
    state.update();
    trajectory.addSuffixWayPoint(state, 0.0);
  }
  std::unordered_map<std::string, double> velocity_limits;
  std::unordered_map<std::string, double> acceleration_limits;
  for (const char * name : kJointNames)
  {
    velocity_limits[name] = kVelocityLimit;
    acceleration_limits[name] = kAccelerationLimit;
  }
  const double generate_started = elapsed();
  trajectory_processing::TimeOptimalTrajectoryGeneration parameterization(kPathTolerance, kResampleDt, kMinAngleChange);
  ++g_totg_calls;
  const bool timed = parameterization.computeTimeStamps(trajectory, velocity_limits, acceleration_limits, 1.0, 1.0);
  const double generate_elapsed = elapsed() - generate_started;
  log << "totg_calls " << g_totg_calls << " returned " << timed << " generate_s " << generate_elapsed << "\n";
  if (g_totg_calls != 1)
  {
    rclcpp::shutdown();
    return fail("CALL_COUNT", "GENERATE");
  }
  if (!timed)
  {
    write_setup(out_dir + "/setup.json", 0, "", "TOTG_RETURNED_FALSE");
    std::ofstream checks(out_dir + "/checks.json");
    checks << "{\n  \"totg_message_check\": \"FAIL\",\n  \"status\": \"TOTG_RETURNED_FALSE\",\n"
           << "  \"compute_time_stamps_calls\": 1,\n  \"generate_s\": " << generate_elapsed << ",\n"
           << "  \"quintic_interior_limits\": {\"status\": \"NOT_EVALUATED\"},\n"
           << "  \"process_accuracy\": {\"status\": \"NOT_EVALUATED\"},\n"
           << "  \"collision\": {\"status\": \"NOT_EVALUATED\"},\n"
           << "  \"correspondence\": {\"status\": \"NOT_EVALUATED\"},\n"
           << "  \"controller_interpolation\": \"NOT_CLAIMED\",\n"
           << "  \"cartesian_path_impossibility\": \"NOT_CLAIMED\"\n}\n";
    rclcpp::shutdown();
    return 3;
  }
  if (generate_elapsed > kGenerateS)
  {
    write_setup(out_dir + "/setup.json", 0, "", "GENERATE_CAP");
    rclcpp::shutdown();
    return fail("GENERATE_CAP", "GENERATE");
  }
  moveit_msgs::msg::RobotTrajectory message;
  const std::vector<std::string> names(std::begin(kJointNames), std::end(kJointNames));
  trajectory.getRobotTrajectoryMsg(message, names);
  const auto & joint_trajectory = message.joint_trajectory;
  std::vector<Point6> points(joint_trajectory.points.size());
  bool message_shape = joint_trajectory.joint_names.size() == 6 && message.multi_dof_joint_trajectory.points.empty() &&
    message.multi_dof_joint_trajectory.joint_names.empty();
  for (int index = 0; index < 6 && message_shape; ++index)
  {
    message_shape = joint_trajectory.joint_names[static_cast<std::size_t>(index)] == kJointNames[index];
  }
  for (std::size_t index = 0; index < points.size(); ++index)
  {
    const auto & source = joint_trajectory.points[index];
    Point6 & point = points[index];
    point.time = static_cast<double>(source.time_from_start.sec) + 1.0e-9 * static_cast<double>(source.time_from_start.nanosec);
    message_shape = message_shape && source.positions.size() == 6 && source.velocities.size() == 6 &&
      source.accelerations.size() == 6;
    if (!message_shape)
    {
      break;
    }
    for (int joint = 0; joint < 6; ++joint)
    {
      point.position[joint] = source.positions[static_cast<std::size_t>(joint)];
      point.velocity[joint] = source.velocities[static_cast<std::size_t>(joint)];
      point.acceleration[joint] = source.accelerations[static_cast<std::size_t>(joint)];
    }
  }
  {
    std::ofstream trajectory_json(out_dir + "/trajectory.json");
    trajectory_json << std::setprecision(17);
    trajectory_json << "{\n  \"message_type\": \"moveit_msgs/msg/RobotTrajectory\",\n"
                    << "  \"source_call\": \"RobotTrajectory::getRobotTrajectoryMsg\",\n"
                    << "  \"joint_trajectory\": {\n    \"header\": {\"frame_id\": \""
                    << joint_trajectory.header.frame_id << "\"},\n    \"joint_names\": [";
    for (std::size_t index = 0; index < joint_trajectory.joint_names.size(); ++index)
    {
      if (index)
      {
        trajectory_json << ", ";
      }
      trajectory_json << "\"" << joint_trajectory.joint_names[index] << "\"";
    }
    trajectory_json << "],\n    \"points\": [";
    for (std::size_t index = 0; index < points.size(); ++index)
    {
      if (index)
      {
        trajectory_json << ", ";
      }
      const auto & source = joint_trajectory.points[index];
      trajectory_json << "{\"time_from_start\": {\"sec\": " << source.time_from_start.sec
                      << ", \"nanosec\": " << source.time_from_start.nanosec << "}, \"positions\": [";
      for (int joint = 0; joint < 6; ++joint)
      {
        if (joint)
        {
          trajectory_json << ", ";
        }
        write_num(trajectory_json, points[index].position[joint]);
      }
      trajectory_json << "], \"velocities\": [";
      for (int joint = 0; joint < 6; ++joint)
      {
        if (joint)
        {
          trajectory_json << ", ";
        }
        write_num(trajectory_json, points[index].velocity[joint]);
      }
      trajectory_json << "], \"accelerations\": [";
      for (int joint = 0; joint < 6; ++joint)
      {
        if (joint)
        {
          trajectory_json << ", ";
        }
        write_num(trajectory_json, points[index].acceleration[joint]);
      }
      trajectory_json << "]}";
    }
    trajectory_json << "]},\n  \"multi_dof_joint_trajectory\": {\"points\": []}\n}\n";
  }
  write_setup(out_dir + "/setup.json", 0, joint_trajectory.header.frame_id, timed ? "TOTG_RETURNED_TRUE" : "TOTG_RETURNED_FALSE");
  const double verify_started = elapsed();
  LimitEval limits_all;
  bool time_ok = message_shape && !points.empty() && points.front().time == 0.0;
  double endpoint_gap = std::numeric_limits<double>::infinity();
  if (message_shape && points.size() >= 2)
  {
    endpoint_gap = 0.0;
    for (int joint = 0; joint < 6; ++joint)
    {
      endpoint_gap = std::max(endpoint_gap, std::abs(points.front().position[joint] - poses.front().joints[joint]));
      endpoint_gap = std::max(endpoint_gap, std::abs(points.back().position[joint] - poses.back().joints[joint]));
    }
    for (std::size_t index = 1; index < points.size(); ++index)
    {
      if (!(points[index].time > points[index - 1].time))
      {
        time_ok = false;
      }
    }
  }
  else
  {
    time_ok = false;
  }
  const bool budget_points = points.size() > static_cast<std::size_t>(kPointCap);
  if (message_shape && !budget_points)
  {
    for (std::size_t index = 1; index < points.size(); ++index)
    {
      if (elapsed() - verify_started > kVerifyS || elapsed() > kOverallS)
      {
        limits_all.finite = false;
        break;
      }
      const double duration = points[index].time - points[index - 1].time;
      Quintic quintic = make_quintic(duration, points[index - 1].position, points[index - 1].velocity,
        points[index - 1].acceleration, points[index].position, points[index].velocity, points[index].acceleration);
      LimitEval interval = evaluate_limits(quintic, limits);
      limits_all.finite = limits_all.finite && interval.finite;
      limits_all.root_fail = limits_all.root_fail || interval.root_fail;
      limits_all.position_fail = limits_all.position_fail || interval.position_fail;
      limits_all.velocity_fail = limits_all.velocity_fail || interval.velocity_fail;
      limits_all.acceleration_fail = limits_all.acceleration_fail || interval.acceleration_fail;
      limits_all.max_velocity = std::max(limits_all.max_velocity, interval.max_velocity);
      limits_all.max_acceleration = std::max(limits_all.max_acceleration, interval.max_acceleration);
      if (interval.root_fail || !interval.finite)
      {
        break;
      }
    }
  }
  const bool limits_pass = message_shape && time_ok && endpoint_gap <= kEndpointRad && limits_all.finite &&
    !limits_all.root_fail && !limits_all.position_fail && !limits_all.velocity_fail && !limits_all.acceleration_fail &&
    !budget_points;
  std::vector<char> covered(943, 0);
  std::vector<FailureRecord> failures;
  std::vector<CrossRecord> crossings;
  int absent = 0;
  int ambiguous = 0;
  int reversal = 0;
  int skipped = 0;
  int pose_mismatch = 0;
  int accuracy_fail = 0;
  int accuracy_checked = 0;
  int self_hit = 0;
  int coupon_hit = 0;
  int collision_error = 0;
  int samples = 0;
  double max_position = 0.0;
  double max_orientation = 0.0;
  bool verify_capped = false;
  bool have_left = false;
  double previous_left = 0.0;
  moveit::core::RobotState state(model);
  if (message_shape && !budget_points)
  {
    for (std::size_t interval = 1; interval < points.size() && !verify_capped; ++interval)
    {
      const double duration = points[interval].time - points[interval - 1].time;
      Quintic quintic = make_quintic(duration, points[interval - 1].position, points[interval - 1].velocity,
        points[interval - 1].acceleration, points[interval].position, points[interval].velocity,
        points[interval].acceleration);
      const int sample_begin = interval == 1 ? 0 : 1;
      for (int sample_index = sample_begin; sample_index <= kSampleDenominator; ++sample_index)
      {
        if (elapsed() - verify_started > kVerifyS || elapsed() > kOverallS - kSaveS)
        {
          verify_capped = true;
          break;
        }
        const double tau = static_cast<double>(sample_index) / static_cast<double>(kSampleDenominator);
        Eigen::VectorXd sample(6);
        for (int joint = 0; joint < 6; ++joint)
        {
          sample[joint] = position_at(quintic, joint, tau);
        }
        ++samples;
        if (!sample.allFinite())
        {
          failures.push_back(FailureRecord{static_cast<int>(interval), sample_index, "NONFINITE", 0.0});
          continue;
        }
        const Assignment assignment = assign_sample(sample, poses);
        if (assignment.type == Assignment::ABSENT)
        {
          ++absent;
          failures.push_back(FailureRecord{static_cast<int>(interval), sample_index, "CORRESPONDENCE_ABSENT", assignment.residual});
        }
        else if (assignment.type == Assignment::AMBIGUOUS)
        {
          ++ambiguous;
          failures.push_back(FailureRecord{static_cast<int>(interval), sample_index, "AMBIGUOUS_CHORD", assignment.residual});
        }
        else
        {
          if (have_left)
          {
            if (assignment.left + 1.0e-9 < previous_left)
            {
              ++reversal;
              failures.push_back(FailureRecord{static_cast<int>(interval), sample_index, "CORRESPONDENCE_REVERSAL", assignment.left});
            }
            else if (assignment.left > previous_left + 1.0 + 1.0e-9)
            {
              ++skipped;
              failures.push_back(FailureRecord{static_cast<int>(interval), sample_index, "BOUNDARY_SKIP", assignment.left});
            }
            else if (assignment.left > previous_left + 1.0e-9)
            {
              crossings.push_back(CrossRecord{static_cast<int>(interval), static_cast<int>(std::floor(previous_left)),
                static_cast<int>(std::floor(assignment.left)), sample_index});
            }
          }
          else if (assignment.left > 0.0)
          {
            ++skipped;
            failures.push_back(FailureRecord{static_cast<int>(interval), sample_index, "BOUNDARY_SKIP", assignment.left});
          }
          have_left = true;
          previous_left = assignment.left;
          if (assignment.type == Assignment::CHORD && assignment.u > kInteriorU && assignment.u < 1.0 - kInteriorU)
          {
            covered[static_cast<std::size_t>(assignment.index)] = 1;
          }
          if (assignment.type == Assignment::VERTEX && assignment.index > 0 && assignment.index < 943)
          {
            const Pose & shared = poses[static_cast<std::size_t>(assignment.index)];
            const double position_gap = (chords[static_cast<std::size_t>(assignment.index - 1)].transition ? shared.position :
              poses[static_cast<std::size_t>(assignment.index)].position - poses[static_cast<std::size_t>(assignment.index)].position).norm();
            (void)position_gap;
            const Eigen::Vector3d left_position = poses[static_cast<std::size_t>(assignment.index)].position;
            const Eigen::Vector3d right_position = poses[static_cast<std::size_t>(assignment.index)].position;
            if ((left_position - right_position).norm() > kEndpointRad)
            {
              ++pose_mismatch;
            }
          }
        }
        state.setJointGroupPositions(group, std::vector<double>(sample.data(), sample.data() + 6));
        state.update();
        collision_detection::CollisionResult self_result;
        collision_detection::CollisionResult coupon_result;
        bool collided = false;
        try
        {
          scene->getCollisionEnv()->checkSelfCollision(request, self_result, state, acm);
          if (!self_result.collision)
          {
            scene->getCollisionEnv()->checkRobotCollision(request, coupon_result, state, acm);
          }
        }
        catch (const std::exception &)
        {
          ++collision_error;
          collided = true;
        }
        if (self_result.collision)
        {
          ++self_hit;
          collided = true;
        }
        else if (coupon_result.collision)
        {
          ++coupon_hit;
          collided = true;
        }
        if (assignment.type == Assignment::ABSENT || assignment.type == Assignment::AMBIGUOUS)
        {
          continue;
        }
        const bool process = assignment.type == Assignment::CHORD ?
          !chords[static_cast<std::size_t>(assignment.index)].transition :
          (assignment.index == 0 ? !chords.front().transition :
            assignment.index == 943 ? !chords.back().transition :
            !chords[static_cast<std::size_t>(assignment.index - 1)].transition ||
            !chords[static_cast<std::size_t>(assignment.index)].transition);
        if (!process)
        {
          continue;
        }
        Eigen::Vector3d target_position;
        Eigen::Quaterniond target_orientation;
        if (assignment.type == Assignment::CHORD)
        {
          const Pose & start = poses[static_cast<std::size_t>(assignment.index)];
          const Pose & end = poses[static_cast<std::size_t>(assignment.index + 1)];
          target_position = (1.0 - assignment.u) * start.position + assignment.u * end.position;
          target_orientation = start.orientation.slerp(assignment.u, end.orientation);
        }
        else
        {
          target_position = poses[static_cast<std::size_t>(assignment.index)].position;
          target_orientation = poses[static_cast<std::size_t>(assignment.index)].orientation;
        }
        const Eigen::Isometry3d actual = state.getGlobalLinkTransform("base_link").inverse() * state.getGlobalLinkTransform("tool0");
        const double position_error = (actual.translation() - target_position).norm();
        const double orientation_error = rotation_error(target_orientation.toRotationMatrix(), actual.rotation());
        ++accuracy_checked;
        if (!std::isfinite(position_error) || !std::isfinite(orientation_error))
        {
          ++accuracy_fail;
          continue;
        }
        max_position = std::max(max_position, position_error);
        max_orientation = std::max(max_orientation, orientation_error);
        if (position_error > kPositionLimitM || orientation_error > kOrientationLimitRad)
        {
          ++accuracy_fail;
        }
        (void)collided;
      }
    }
  }
  int missing = 0;
  int missing_process = 0;
  int missing_transition = 0;
  for (int chord = 0; chord < 943; ++chord)
  {
    if (!covered[static_cast<std::size_t>(chord)])
    {
      ++missing;
      if (chords[static_cast<std::size_t>(chord)].transition)
      {
        ++missing_transition;
      }
      else
      {
        ++missing_process;
      }
    }
  }
  const bool correspondence_pass = !budget_points && !verify_capped && absent == 0 && ambiguous == 0 && reversal == 0 &&
    skipped == 0 && pose_mismatch == 0 && missing == 0 && message_shape;
  const bool accuracy_pass = correspondence_pass && accuracy_fail == 0 && accuracy_checked > 0;
  const bool collision_pass = correspondence_pass && self_hit == 0 && coupon_hit == 0 && collision_error == 0 && samples > 0;
  const bool overall = limits_pass && correspondence_pass && accuracy_pass && collision_pass && g_totg_calls == 1;
  {
    std::ofstream correspondence(out_dir + "/correspondence.json");
    correspondence << std::setprecision(17);
    correspondence << "{\n  \"rule\": \"CONSERVATIVE_CHORD_PROJECTION\",\n  \"auto_correction\": false,\n"
                   << "  \"cartesian_path_impossibility\": \"NOT_CLAIMED\",\n"
                   << "  \"status\": \"" << (correspondence_pass ? "PASS" : "FAIL") << "\",\n"
                   << "  \"absent\": " << absent << ",\n  \"ambiguous\": " << ambiguous << ",\n  \"reversal\": " << reversal
                   << ",\n  \"boundary_skip\": " << skipped << ",\n  \"endpoint_pose_mismatch\": " << pose_mismatch
                   << ",\n  \"missing_chord\": " << missing << ",\n  \"missing_process\": " << missing_process
                   << ",\n  \"missing_transition\": " << missing_transition << ",\n  \"samples\": " << samples
                   << ",\n  \"crossings\": [";
    const std::size_t crossing_limit = std::min<std::size_t>(crossings.size(), 2000);
    for (std::size_t index = 0; index < crossing_limit; ++index)
    {
      if (index)
      {
        correspondence << ", ";
      }
      correspondence << "{\"interval\": " << crossings[index].interval << ", \"from_chord\": " << crossings[index].from_chord
                     << ", \"to_chord\": " << crossings[index].to_chord << ", \"between_k\": " << crossings[index].between_k << "}";
    }
    correspondence << "],\n  \"crossings_recorded\": " << crossing_limit << ",\n  \"crossings_total\": " << crossings.size()
                   << ",\n  \"failures\": [";
    const std::size_t failure_limit = std::min<std::size_t>(failures.size(), 500);
    for (std::size_t index = 0; index < failure_limit; ++index)
    {
      if (index)
      {
        correspondence << ", ";
      }
      correspondence << "{\"interval\": " << failures[index].interval << ", \"sample\": " << failures[index].sample
                     << ", \"reason\": \"" << failures[index].reason << "\", \"value\": ";
      write_num(correspondence, failures[index].residual);
      correspondence << "}";
    }
    correspondence << "],\n  \"failures_recorded\": " << failure_limit << ",\n  \"failures_total\": " << failures.size()
                   << "\n}\n";
  }
  {
    std::ofstream checks(out_dir + "/checks.json");
    checks << std::setprecision(17);
    checks << "{\n"
           << "  \"totg_message_check\": \"" << (overall ? "PASS" : "FAIL") << "\",\n"
           << "  \"compute_time_stamps_calls\": " << g_totg_calls << ",\n"
           << "  \"totg_returned\": " << (timed ? "true" : "false") << ",\n"
           << "  \"generate_s\": " << generate_elapsed << ",\n"
           << "  \"point_count\": " << points.size() << ",\n"
           << "  \"budget_points\": " << (budget_points ? "true" : "false") << ",\n"
           << "  \"verify_capped\": " << (verify_capped ? "true" : "false") << ",\n"
           << "  \"message_shape\": " << (message_shape ? "true" : "false") << ",\n"
           << "  \"time_monotonic_from_zero\": " << (time_ok ? "true" : "false") << ",\n"
           << "  \"endpoint_gap_rad\": " << endpoint_gap << ",\n"
           << "  \"interpolation\": \"PER_JOINT_QUINTIC_HERMITE\",\n"
           << "  \"controller_interpolation\": \"NOT_CLAIMED\",\n"
           << "  \"continuous_collision\": \"NOT_CLAIMED\",\n"
           << "  \"jerk_limit\": \"NOT_CHECKED\",\n"
           << "  \"quintic_interior_limits\": {\n"
           << "    \"status\": \"" << (limits_pass ? "PASS" : "FAIL") << "\",\n"
           << "    \"finite\": " << (limits_all.finite ? "true" : "false") << ",\n"
           << "    \"root_fail\": " << (limits_all.root_fail ? "true" : "false") << ",\n"
           << "    \"position_fail\": " << (limits_all.position_fail ? "true" : "false") << ",\n"
           << "    \"velocity_fail\": " << (limits_all.velocity_fail ? "true" : "false") << ",\n"
           << "    \"acceleration_fail\": " << (limits_all.acceleration_fail ? "true" : "false") << ",\n"
           << "    \"max_abs_velocity_rad_s\": " << limits_all.max_velocity << ",\n"
           << "    \"max_abs_acceleration_rad_s2\": " << limits_all.max_acceleration << "\n  },\n"
           << "  \"process_accuracy\": {\n"
           << "    \"status\": \"" << (budget_points || verify_capped ? "NOT_EVALUATED" : accuracy_pass ? "PASS" : "FAIL") << "\",\n"
           << "    \"samples_checked\": " << accuracy_checked << ",\n"
           << "    \"fail_count\": " << accuracy_fail << ",\n"
           << "    \"max_position_error_m\": " << max_position << ",\n"
           << "    \"max_orientation_error_rad\": " << max_orientation << ",\n"
           << "    \"position_limit_m\": 0.001,\n"
           << "    \"orientation_limit_rad\": 0.01\n  },\n"
           << "  \"collision\": {\n"
           << "    \"status\": \"" << (budget_points || verify_capped ? "NOT_EVALUATED" : collision_pass ? "PASS" : "FAIL") << "\",\n"
           << "    \"samples\": " << samples << ",\n"
           << "    \"self_hit\": " << self_hit << ",\n"
           << "    \"coupon_hit\": " << coupon_hit << ",\n"
           << "    \"error\": " << collision_error << "\n  },\n"
           << "  \"correspondence\": {\n"
           << "    \"status\": \"" << (correspondence_pass ? "PASS" : "FAIL") << "\",\n"
           << "    \"absent\": " << absent << ",\n"
           << "    \"ambiguous\": " << ambiguous << ",\n"
           << "    \"reversal\": " << reversal << ",\n"
           << "    \"boundary_skip\": " << skipped << ",\n"
           << "    \"missing_chord\": " << missing << ",\n"
           << "    \"cartesian_path_impossibility\": \"NOT_CLAIMED\"\n  }\n}\n";
  }
  {
    std::ofstream report(out_dir + "/report.md");
    report << std::setprecision(17);
    report << "# TOTG 단일 실행\n\n";
    report << "```text\nTOTG_MESSAGE_CHECK = " << (overall ? "PASS" : "FAIL") << "\n";
    report << "CALLS = " << g_totg_calls << "\n";
    report << "INTERPOLATION = PER_JOINT_QUINTIC_HERMITE\n";
    report << "CONTROLLER_INTERPOLATION = NOT_CLAIMED\n";
    report << "QUINTIC_INTERIOR_LIMITS = " << (limits_pass ? "PASS" : "FAIL") << "\n";
    report << "CORRESPONDENCE = " << (correspondence_pass ? "PASS" : "FAIL") << "\n";
    report << "PROCESS_ACCURACY = " << (budget_points || verify_capped ? "NOT_EVALUATED" : accuracy_pass ? "PASS" : "FAIL") << "\n";
    report << "COLLISION = " << (budget_points || verify_capped ? "NOT_EVALUATED" : collision_pass ? "PASS" : "FAIL") << "\n";
    report << "POINT_COUNT = " << points.size() << "\n";
    report << "GENERATE_S = " << generate_elapsed << "\n";
    report << "CARTESIAN_PATH_IMPOSSIBILITY = NOT_CLAIMED\n```\n\n";
    report << "이 검사는 planning-reference UR5e의 저장 메시지를 5차 Hermite로 해석한 결과다. ";
    report << "실제 컨트롤러가 이 보간을 쓴다고 말하지 않는다. ";
    report << "대응 실패는 Cartesian 경로 자체가 불가능하다는 뜻이 아니다.\n\n";
    report << "QUINTIC_INTERIOR_LIMITS와 공정 정확성·충돌은 별도 항목이다. ";
    report << "속도 최대 절대값 " << limits_all.max_velocity << " rad/s, 가속도 최대 절대값 "
           << limits_all.max_acceleration << " rad/s^2.\n\n";
    report << "대응: absent " << absent << ", ambiguous " << ambiguous << ", reversal " << reversal
           << ", boundary_skip " << skipped << ", missing_chord " << missing << ".\n\n";
    report << "공정 표본 " << accuracy_checked << ", 실패 " << accuracy_fail << ", 위치 최대 " << max_position
           << " m, 방향 최대 " << max_orientation << " rad.\n\n";
    report << "충돌 표본 " << samples << ", 자체 " << self_hit << ", coupon " << coupon_hit << ", error " << collision_error << ".\n\n";
    report << "P-022, 기하 경로 manifest, 분사 데모는 이 실행에서 수정하지 않았다.\n";
  }
  {
    std::ofstream command(out_dir + "/command.txt");
    command << "ur_analytic_ik_totg_trajectory";
    for (int index = 1; index < argc; ++index)
    {
      command << " " << argv[index];
    }
    command << "\n";
  }
  rclcpp::shutdown();
  std::cout << (overall ? "PASS" : "FAIL") << " points " << points.size() << " calls " << g_totg_calls << "\n";
  return overall ? 0 : 3;
}
