// 채택 관절 직선을 유지하는 구간별 5차 시간 스케일이다.
// TOTG는 호출하지 않는다. 검사 대상은 저장 후 다시 읽은 메시지다.

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
  std::int32_t sec = 0;
  std::uint32_t nanosec = 0;
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
  // 정지-정지 직선. T=1, Δ=1. 극값은 15/8과 (10/3)√3이다.
  limits.col(0).setConstant(-10.0);
  limits.col(1).setConstant(10.0);
  set_joint0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  Quintic rest = make_quintic(1.0, p0, v0, a0, p1, v1, a1);
  const double sdot = 15.0 / 8.0;
  const double sddot = (10.0 / 3.0) * std::sqrt(3.0);
  const double tau_low = (3.0 - std::sqrt(3.0)) / 6.0;
  const double tau_high = (3.0 + std::sqrt(3.0)) / 6.0;
  if (!rest.finite || std::abs(position_at(rest, 0, 0.5) - 0.5) > 1.0e-12 ||
      std::abs(std::abs(d1(rest.c[0], 0.5)) - sdot) > 1.0e-12 ||
      std::abs(std::abs(d2(rest.c[0], tau_low)) - sddot) > 1.0e-12 ||
      std::abs(std::abs(d2(rest.c[0], tau_high)) - sddot) > 1.0e-12 ||
      std::abs(d1(rest.c[0], 0.0)) > 1.0e-12 || std::abs(d1(rest.c[0], 1.0)) > 1.0e-12 ||
      std::abs(d2(rest.c[0], 0.0)) > 1.0e-12 || std::abs(d2(rest.c[0], 1.0)) > 1.0e-12)
  {
    *detail = "rest_to_rest_scale";
    return false;
  }
  LimitEval rest_limits = evaluate_limits(rest, limits);
  if (!rest_limits.finite || rest_limits.root_fail)
  {
    *detail = "rest_to_rest_roots";
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
         << "  \"algorithm\": \"PER_SEGMENT_QUINTIC_TIME_SCALING\",\n"
         << "  \"time_formula\": \"max_j max((15/8)|d_j|/v_j, sqrt((10/3)sqrt(3)|d_j|/a_j))\",\n"
         << "  \"duration_storage\": \"INTEGER_NANOSECOND_CEIL\",\n"
         << "  \"velocity_limit_rad_s\": " << kVelocityLimit << ",\n"
         << "  \"velocity_limit_class\": \"URDF_LIMIT\",\n"
         << "  \"acceleration_limit_rad_s2\": " << kAccelerationLimit << ",\n"
         << "  \"acceleration_limit_class\": \"MOVEIT_CONFIGURATION\",\n"
         << "  \"interpolation\": \"PER_JOINT_QUINTIC_HERMITE\",\n"
         << "  \"controller_interpolation\": \"NOT_CLAIMED\",\n"
         << "  \"spray_efficiency\": \"NOT_CLAIMED\",\n"
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
    checks << "{\"quintic_timescale_check\": \"FAIL\", \"phase\": \"" << phase << "\", \"status\": \"" << status
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
    hermite << "{\"status\": \"PASS\", \"detail\": \"known quintic midpoint, interior velocity root, zero jerk, nonfinite rejected, velocity violation detected, rest-to-rest scale\"}\n";
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

  const double generate_started = elapsed();
  const double kSdotMax = 15.0 / 8.0;
  const double kSddotMax = (10.0 / 3.0) * std::sqrt(3.0);
  const std::int64_t kMaxSecond = 2147483647LL;
  std::vector<std::int64_t> scheduled_ns(944, 0);
  std::int64_t cursor_ns = 0;
  for (int index = 0; index < 943; ++index)
  {
    double lower = 0.0;
    bool moved = false;
    for (int joint = 0; joint < 6; ++joint)
    {
      const double delta = poses[static_cast<std::size_t>(index + 1)].joints[joint] -
        poses[static_cast<std::size_t>(index)].joints[joint];
      if (!std::isfinite(delta))
      {
        rclcpp::shutdown();
        return fail("NONFINITE", "GENERATE");
      }
      const double magnitude = std::abs(delta);
      if (magnitude == 0.0)
      {
        continue;
      }
      moved = true;
      const double velocity_bound = kSdotMax * magnitude / kVelocityLimit;
      const double acceleration_bound = std::sqrt(kSddotMax * magnitude / kAccelerationLimit);
      if (!std::isfinite(velocity_bound) || !std::isfinite(acceleration_bound))
      {
        rclcpp::shutdown();
        return fail("NONFINITE", "GENERATE");
      }
      lower = std::max(lower, std::max(velocity_bound, acceleration_bound));
    }
    if (!moved)
    {
      rclcpp::shutdown();
      return fail("ZERO_CHORD", "GENERATE");
    }
    const long double product = static_cast<long double>(lower) * 1000000000.0L;
    if (!std::isfinite(lower) || product > static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
    {
      rclcpp::shutdown();
      return fail("TIME_RANGE", "GENERATE");
    }
    long double ceiled = std::ceil(product);
    if (ceiled > static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
    {
      rclcpp::shutdown();
      return fail("TIME_RANGE", "GENERATE");
    }
    std::int64_t duration_ns = static_cast<std::int64_t>(ceiled);
    while (duration_ns < std::numeric_limits<std::int64_t>::max() &&
           static_cast<long double>(duration_ns) < product)
    {
      ++duration_ns;
    }
    if (duration_ns < 1)
    {
      rclcpp::shutdown();
      return fail("DT_TOO_SMALL", "GENERATE");
    }
    if (cursor_ns > std::numeric_limits<std::int64_t>::max() - duration_ns)
    {
      rclcpp::shutdown();
      return fail("TIME_RANGE", "GENERATE");
    }
    cursor_ns += duration_ns;
    if (cursor_ns / 1000000000LL > kMaxSecond)
    {
      rclcpp::shutdown();
      return fail("TIME_RANGE", "GENERATE");
    }
    scheduled_ns[static_cast<std::size_t>(index + 1)] = cursor_ns;
  }
  for (int index = 1; index < 944; ++index)
  {
    if (!(scheduled_ns[static_cast<std::size_t>(index)] > scheduled_ns[static_cast<std::size_t>(index - 1)]))
    {
      rclcpp::shutdown();
      return fail("TIME_NONINCREASING", "GENERATE");
    }
  }
  robot_trajectory::RobotTrajectory trajectory(model, "ur_manipulator");
  std::vector<double> zeros(6, 0.0);
  for (int index = 0; index < 944; ++index)
  {
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    std::vector<double> values(poses[static_cast<std::size_t>(index)].joints.data(),
      poses[static_cast<std::size_t>(index)].joints.data() + 6);
    state.setJointGroupPositions(group, values);
    state.setJointGroupVelocities(group, zeros);
    state.setJointGroupAccelerations(group, zeros);
    state.update();
    const double dt = index == 0 ? 0.0 :
      static_cast<double>(scheduled_ns[static_cast<std::size_t>(index)] - scheduled_ns[static_cast<std::size_t>(index - 1)]) * 1.0e-9;
    trajectory.addSuffixWayPoint(state, dt);
  }
  if (g_totg_calls != 0)
  {
    rclcpp::shutdown();
    return fail("CALL_COUNT", "GENERATE");
  }
  if (elapsed() - generate_started > kGenerateS)
  {
    rclcpp::shutdown();
    return fail("GENERATE_CAP", "GENERATE");
  }
  moveit_msgs::msg::RobotTrajectory message;
  const std::vector<std::string> names(std::begin(kJointNames), std::end(kJointNames));
  trajectory.getRobotTrajectoryMsg(message, names);
  if (message.joint_trajectory.points.size() != 944)
  {
    rclcpp::shutdown();
    return fail("MESSAGE_SHAPE", "GENERATE");
  }
  for (int index = 0; index < 944; ++index)
  {
    const std::int64_t stamp = scheduled_ns[static_cast<std::size_t>(index)];
    message.joint_trajectory.points[static_cast<std::size_t>(index)].time_from_start.sec =
      static_cast<std::int32_t>(stamp / 1000000000LL);
    message.joint_trajectory.points[static_cast<std::size_t>(index)].time_from_start.nanosec =
      static_cast<std::uint32_t>(stamp % 1000000000LL);
  }
  const double generate_elapsed = elapsed() - generate_started;
  log << "totg_calls " << g_totg_calls << " generate_s " << generate_elapsed
      << " total_ns " << cursor_ns << "\n";
  const auto & joint_trajectory = message.joint_trajectory;
  {
    std::ofstream trajectory_json(out_dir + "/trajectory.json");
    trajectory_json << std::setprecision(17);
    trajectory_json << "{\n  \"message_type\": \"moveit_msgs/msg/RobotTrajectory\",\n"
                    << "  \"source_call\": \"RobotTrajectory::getRobotTrajectoryMsg\",\n"
                    << "  \"time_from_start_source\": \"INTEGER_NANOSECOND_ACCUMULATION\",\n"
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
    for (std::size_t index = 0; index < joint_trajectory.points.size(); ++index)
    {
      if (index)
      {
        trajectory_json << ", ";
      }
      const auto & source = joint_trajectory.points[index];
      trajectory_json << "{\"time_from_start\": {\"sec\": " << source.time_from_start.sec
                      << ", \"nanosec\": " << source.time_from_start.nanosec << "}, \"positions\": [";
      for (std::size_t joint = 0; joint < source.positions.size(); ++joint)
      {
        if (joint)
        {
          trajectory_json << ", ";
        }
        write_num(trajectory_json, source.positions[joint]);
      }
      trajectory_json << "], \"velocities\": [";
      for (std::size_t joint = 0; joint < source.velocities.size(); ++joint)
      {
        if (joint)
        {
          trajectory_json << ", ";
        }
        write_num(trajectory_json, source.velocities[joint]);
      }
      trajectory_json << "], \"accelerations\": [";
      for (std::size_t joint = 0; joint < source.accelerations.size(); ++joint)
      {
        if (joint)
        {
          trajectory_json << ", ";
        }
        write_num(trajectory_json, source.accelerations[joint]);
      }
      trajectory_json << "]}";
    }
    trajectory_json << "]},\n  \"multi_dof_joint_trajectory\": {\"points\": []}\n}\n";
  }
  const std::string saved = read_text(out_dir + "/trajectory.json");
  const std::string trajectory_sha = sha256_file(out_dir + "/trajectory.json");
  std::vector<Point6> points;
  std::size_t scan = 0;
  bool reread_ok = saved.find("\"source_call\": \"RobotTrajectory::getRobotTrajectoryMsg\"") != std::string::npos;
  while (reread_ok)
  {
    const std::size_t stamp_at = saved.find("\"time_from_start\"", scan);
    if (stamp_at == std::string::npos)
    {
      break;
    }
    const std::size_t sec_at = saved.find("\"sec\":", stamp_at);
    const std::size_t nano_at = saved.find("\"nanosec\":", stamp_at);
    if (sec_at == std::string::npos || nano_at == std::string::npos)
    {
      reread_ok = false;
      break;
    }
    char * sec_end = nullptr;
    char * nano_end = nullptr;
    const long sec_value = std::strtol(saved.c_str() + sec_at + 6, &sec_end, 10);
    const unsigned long nano_value = std::strtoul(saved.c_str() + nano_at + 10, &nano_end, 10);
    if (sec_end == saved.c_str() + sec_at + 6 || nano_end == saved.c_str() + nano_at + 10)
    {
      reread_ok = false;
      break;
    }
    Point6 point;
    point.sec = static_cast<std::int32_t>(sec_value);
    point.nanosec = static_cast<std::uint32_t>(nano_value);
    point.time = static_cast<double>(point.sec) + 1.0e-9 * static_cast<double>(point.nanosec);
    auto read6 = [&](const char * key, double * destination) {
      const std::size_t key_at = saved.find(key, static_cast<std::size_t>(nano_end - saved.c_str()));
      const std::size_t bracket = key_at == std::string::npos ? std::string::npos : saved.find('[', key_at);
      if (bracket == std::string::npos)
      {
        return false;
      }
      std::size_t number_at = bracket + 1;
      for (int joint = 0; joint < 6; ++joint)
      {
        while (number_at < saved.size() &&
               (saved[number_at] == ' ' || saved[number_at] == ',' || saved[number_at] == '\n'))
        {
          ++number_at;
        }
        char * number_end = nullptr;
        destination[joint] = std::strtod(saved.c_str() + number_at, &number_end);
        if (number_end == saved.c_str() + number_at || !std::isfinite(destination[joint]))
        {
          return false;
        }
        number_at = static_cast<std::size_t>(number_end - saved.c_str());
      }
      scan = number_at;
      return true;
    };
    reread_ok = read6("\"positions\"", point.position) && read6("\"velocities\"", point.velocity) &&
      read6("\"accelerations\"", point.acceleration);
    if (!reread_ok)
    {
      break;
    }
    points.push_back(point);
  }
  bool message_shape = reread_ok && points.size() == 944 && joint_trajectory.joint_names.size() == 6 &&
    message.multi_dof_joint_trajectory.points.empty();
  for (int index = 0; index < 6 && message_shape; ++index)
  {
    message_shape = joint_trajectory.joint_names[static_cast<std::size_t>(index)] == kJointNames[index];
  }
  bool rest_ok = message_shape;
  bool time_ok = message_shape && points.front().sec == 0 && points.front().nanosec == 0;
  bool stored_time_match = message_shape;
  double position_gap = 0.0;
  if (message_shape)
  {
    for (int index = 0; index < 944; ++index)
    {
      const Point6 & point = points[static_cast<std::size_t>(index)];
      const std::int64_t read_ns = static_cast<std::int64_t>(point.sec) * 1000000000LL +
        static_cast<std::int64_t>(point.nanosec);
      if (read_ns != scheduled_ns[static_cast<std::size_t>(index)])
      {
        stored_time_match = false;
      }
      if (index > 0 && !(read_ns > static_cast<std::int64_t>(points[static_cast<std::size_t>(index - 1)].sec) * 1000000000LL +
            static_cast<std::int64_t>(points[static_cast<std::size_t>(index - 1)].nanosec)))
      {
        time_ok = false;
      }
      for (int joint = 0; joint < 6; ++joint)
      {
        if (point.velocity[joint] != 0.0 || point.acceleration[joint] != 0.0)
        {
          rest_ok = false;
        }
        position_gap = std::max(position_gap,
          std::abs(point.position[joint] - poses[static_cast<std::size_t>(index)].joints[joint]));
      }
    }
  }
  else
  {
    time_ok = false;
    rest_ok = false;
  }
  write_setup(out_dir + "/setup.json", 0, joint_trajectory.header.frame_id, "MESSAGE_WRITTEN");
  const double verify_started = elapsed();
  LimitEval limits_all;
  const bool budget_points = points.size() > static_cast<std::size_t>(kPointCap);
  bool limits_capped = false;
  if (message_shape && !budget_points && time_ok)
  {
    for (std::size_t index = 1; index < points.size(); ++index)
    {
      if (elapsed() - verify_started > kVerifyS || elapsed() > kOverallS - kSaveS)
      {
        limits_capped = true;
        break;
      }
      const std::int64_t dt_ns = (static_cast<std::int64_t>(points[index].sec) * 1000000000LL +
          static_cast<std::int64_t>(points[index].nanosec)) -
        (static_cast<std::int64_t>(points[index - 1].sec) * 1000000000LL +
          static_cast<std::int64_t>(points[index - 1].nanosec));
      const double duration = static_cast<double>(dt_ns) * 1.0e-9;
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
  const bool interior_pass = message_shape && time_ok && !limits_capped && limits_all.finite && !limits_all.root_fail &&
    !limits_all.position_fail && !limits_all.velocity_fail && !limits_all.acceleration_fail && !budget_points;
  const bool limits_pass = interior_pass && rest_ok && stored_time_match && position_gap <= kEndpointRad;
  std::vector<FailureRecord> failures;
  int path_deviation = 0;
  int missing = points.size() == 944 ? 0 : 943;
  int accuracy_fail = 0;
  int accuracy_checked = 0;
  int self_hit = 0;
  int coupon_hit = 0;
  int collision_error = 0;
  int samples = 0;
  double max_position = 0.0;
  double max_orientation = 0.0;
  bool verify_capped = limits_capped;
  std::vector<int> interval_deviation(943, 0);
  std::vector<int> interval_accuracy_fail(943, 0);
  std::vector<int> interval_accuracy_checked(943, 0);
  std::vector<int> interval_self(943, 0);
  std::vector<int> interval_coupon(943, 0);
  moveit::core::RobotState state(model);
  auto scalar_s = [](double tau) {
    const double tau2 = tau * tau;
    const double tau3 = tau2 * tau;
    return 10.0 * tau3 - 15.0 * tau2 * tau2 + 6.0 * tau2 * tau3;
  };
  if (message_shape && !budget_points && time_ok)
  {
    for (std::size_t interval = 1; interval < points.size() && !verify_capped; ++interval)
    {
      const std::int64_t dt_ns = (static_cast<std::int64_t>(points[interval].sec) * 1000000000LL +
          static_cast<std::int64_t>(points[interval].nanosec)) -
        (static_cast<std::int64_t>(points[interval - 1].sec) * 1000000000LL +
          static_cast<std::int64_t>(points[interval - 1].nanosec));
      const double duration = static_cast<double>(dt_ns) * 1.0e-9;
      Quintic quintic = make_quintic(duration, points[interval - 1].position, points[interval - 1].velocity,
        points[interval - 1].acceleration, points[interval].position, points[interval].velocity,
        points[interval].acceleration);
      const int sample_begin = interval == 1 ? 0 : 1;
      const bool process = !chords[interval - 1].transition;
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
        const int chord = static_cast<int>(interval - 1);
        const Eigen::VectorXd delta = poses[static_cast<std::size_t>(chord + 1)].joints - poses[static_cast<std::size_t>(chord)].joints;
        const double length2 = delta.squaredNorm();
        double projected_u = 0.0;
        Eigen::VectorXd projected = poses[static_cast<std::size_t>(chord)].joints;
        if (length2 > 1.0e-24)
        {
          projected_u = std::clamp((sample - poses[static_cast<std::size_t>(chord)].joints).dot(delta) / length2, 0.0, 1.0);
          projected = poses[static_cast<std::size_t>(chord)].joints + projected_u * delta;
        }
        const double residual = (sample - projected).norm();
        const double parameter_gap = std::abs(projected_u - scalar_s(tau));
        const bool deviation = !sample.allFinite() || residual > 1.0e-8 || parameter_gap > 1.0e-8;
        if (deviation)
        {
          ++path_deviation;
          ++interval_deviation[static_cast<std::size_t>(chord)];
          if (failures.size() < 500)
          {
            failures.push_back(FailureRecord{static_cast<int>(interval), sample_index, "PATH_DEVIATION", residual});
          }
        }
        state.setJointGroupPositions(group, std::vector<double>(sample.data(), sample.data() + 6));
        state.update();
        collision_detection::CollisionResult self_result;
        collision_detection::CollisionResult coupon_result;
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
        }
        if (self_result.collision)
        {
          ++self_hit;
          ++interval_self[static_cast<std::size_t>(chord)];
        }
        else if (coupon_result.collision)
        {
          ++coupon_hit;
          ++interval_coupon[static_cast<std::size_t>(chord)];
        }
        if (!process || deviation)
        {
          continue;
        }
        const double parameter = scalar_s(tau);
        const Pose & start = poses[static_cast<std::size_t>(chord)];
        const Pose & end = poses[static_cast<std::size_t>(chord + 1)];
        const Eigen::Vector3d target_position = (1.0 - parameter) * start.position + parameter * end.position;
        const Eigen::Quaterniond target_orientation = start.orientation.slerp(parameter, end.orientation);
        const Eigen::Isometry3d actual = state.getGlobalLinkTransform("base_link").inverse() * state.getGlobalLinkTransform("tool0");
        const double position_error = (actual.translation() - target_position).norm();
        const double orientation_error = rotation_error(target_orientation.toRotationMatrix(), actual.rotation());
        ++accuracy_checked;
        ++interval_accuracy_checked[static_cast<std::size_t>(chord)];
        if (std::isfinite(position_error))
        {
          max_position = std::max(max_position, position_error);
        }
        if (std::isfinite(orientation_error))
        {
          max_orientation = std::max(max_orientation, orientation_error);
        }
        if (!std::isfinite(position_error) || !std::isfinite(orientation_error) ||
            position_error > kPositionLimitM || orientation_error > kOrientationLimitRad)
        {
          ++accuracy_fail;
          ++interval_accuracy_fail[static_cast<std::size_t>(chord)];
        }
      }
    }
  }
  if (verify_capped)
  {
    missing = 943;
  }
  const bool path_pass = message_shape && !budget_points && !verify_capped && path_deviation == 0 && missing == 0;
  const bool accuracy_pass = !budget_points && !verify_capped && accuracy_fail == 0 && accuracy_checked > 0;
  const bool collision_pass = !budget_points && !verify_capped && self_hit == 0 && coupon_hit == 0 &&
    collision_error == 0 && samples > 0;
  const bool overall = limits_pass && path_pass && accuracy_pass && collision_pass && g_totg_calls == 0;
  {
    std::ofstream correspondence(out_dir + "/correspondence.json");
    correspondence << std::setprecision(17);
    correspondence << "{\n  \"rule\": \"DIRECT_CHORD_INDEX\",\n  \"nearest_assignment\": false,\n"
                   << "  \"auto_correction\": false,\n  \"cartesian_path_impossibility\": \"NOT_CLAIMED\",\n"
                   << "  \"status\": \"" << (verify_capped || budget_points ? "NOT_EVALUATED" : path_pass ? "PASS" : "FAIL")
                   << "\",\n  \"path_deviation\": " << path_deviation << ",\n  \"missing_chord\": " << missing
                   << ",\n  \"samples\": " << samples << ",\n  \"failures\": [";
    for (std::size_t index = 0; index < failures.size(); ++index)
    {
      if (index)
      {
        correspondence << ", ";
      }
      correspondence << "{\"interval\": " << failures[index].interval << ", \"sample\": " << failures[index].sample
                     << ", \"reason\": \"" << failures[index].reason << "\", \"residual_rad\": ";
      write_num(correspondence, failures[index].residual);
      correspondence << "}";
    }
    correspondence << "],\n  \"failures_recorded\": " << failures.size() << ",\n  \"failures_total\": " << path_deviation
                   << "\n}\n";
  }
  {
    std::ofstream intervals(out_dir + "/intervals.json");
    intervals << std::setprecision(17);
    intervals << "{\n  \"duration_ns_source\": \"INTEGER_NANOSECOND_CEIL\",\n  \"intervals\": [";
    for (int index = 0; index < 943; ++index)
    {
      if (index)
      {
        intervals << ", ";
      }
      const std::int64_t duration_ns = scheduled_ns[static_cast<std::size_t>(index + 1)] -
        scheduled_ns[static_cast<std::size_t>(index)];
      intervals << "{\"index\": " << index << ", \"transition\": " << (chords[static_cast<std::size_t>(index)].transition ? "true" : "false")
                << ", \"duration_ns\": " << duration_ns
                << ", \"path_deviation\": " << interval_deviation[static_cast<std::size_t>(index)]
                << ", \"accuracy_checked\": " << interval_accuracy_checked[static_cast<std::size_t>(index)]
                << ", \"accuracy_fail\": " << interval_accuracy_fail[static_cast<std::size_t>(index)]
                << ", \"self_hit\": " << interval_self[static_cast<std::size_t>(index)]
                << ", \"coupon_hit\": " << interval_coupon[static_cast<std::size_t>(index)] << "}";
    }
    intervals << "]\n}\n";
  }
  {
    std::ofstream checks(out_dir + "/checks.json");
    checks << std::setprecision(17);
    checks << "{\n"
           << "  \"quintic_timescale_check\": \"" << (overall ? "PASS" : "FAIL") << "\",\n"
           << "  \"compute_time_stamps_calls\": " << g_totg_calls << ",\n"
           << "  \"generate_s\": " << generate_elapsed << ",\n"
           << "  \"point_count\": " << points.size() << ",\n"
           << "  \"total_duration_ns\": " << cursor_ns << ",\n"
           << "  \"trajectory_sha256\": \"" << trajectory_sha << "\",\n"
           << "  \"budget_points\": " << (budget_points ? "true" : "false") << ",\n"
           << "  \"verify_capped\": " << (verify_capped ? "true" : "false") << ",\n"
           << "  \"message_shape\": " << (message_shape ? "true" : "false") << ",\n"
           << "  \"reread_message\": " << (reread_ok ? "true" : "false") << ",\n"
           << "  \"time_monotonic_from_zero\": " << (time_ok ? "true" : "false") << ",\n"
           << "  \"stored_time_match\": " << (stored_time_match ? "true" : "false") << ",\n"
           << "  \"rest_boundary\": " << (rest_ok ? "true" : "false") << ",\n"
           << "  \"position_gap_rad\": " << position_gap << ",\n"
           << "  \"interpolation\": \"PER_JOINT_QUINTIC_HERMITE\",\n"
           << "  \"controller_interpolation\": \"NOT_CLAIMED\",\n"
           << "  \"spray_efficiency\": \"NOT_CLAIMED\",\n"
           << "  \"continuous_collision\": \"NOT_CLAIMED\",\n"
           << "  \"jerk_limit\": \"NOT_CHECKED\",\n"
           << "  \"quintic_interior_limits\": {\n"
           << "    \"status\": \"" << (budget_points || limits_capped || !message_shape || !time_ok ? "NOT_EVALUATED" : interior_pass ? "PASS" : "FAIL") << "\",\n"
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
           << "  \"path\": {\n"
           << "    \"status\": \"" << (budget_points || verify_capped ? "NOT_EVALUATED" : path_pass ? "PASS" : "FAIL") << "\",\n"
           << "    \"rule\": \"DIRECT_CHORD_INDEX\",\n"
           << "    \"path_deviation\": " << path_deviation << ",\n"
           << "    \"missing_chord\": " << missing << ",\n"
           << "    \"cartesian_path_impossibility\": \"NOT_CLAIMED\"\n  }\n}\n";
  }
  {
    std::ofstream report(out_dir + "/report.md");
    report << std::setprecision(17);
    report << "# 구간별 5차 시간 스케일링\n\n";
    report << "```text\nQUINTIC_TIMESCALE_CHECK = " << (overall ? "PASS" : "FAIL") << "\n";
    report << "TOTG_CALLS = " << g_totg_calls << "\n";
    report << "INTERPOLATION = PER_JOINT_QUINTIC_HERMITE\n";
    report << "CONTROLLER_INTERPOLATION = NOT_CLAIMED\n";
    report << "SPRAY_EFFICIENCY = NOT_CLAIMED\n";
    report << "QUINTIC_INTERIOR_LIMITS = " << (budget_points || limits_capped || !message_shape || !time_ok ? "NOT_EVALUATED" : interior_pass ? "PASS" : "FAIL") << "\n";
    report << "PATH = " << (budget_points || verify_capped ? "NOT_EVALUATED" : path_pass ? "PASS" : "FAIL") << "\n";
    report << "PROCESS_ACCURACY = " << (budget_points || verify_capped ? "NOT_EVALUATED" : accuracy_pass ? "PASS" : "FAIL") << "\n";
    report << "COLLISION = " << (budget_points || verify_capped ? "NOT_EVALUATED" : collision_pass ? "PASS" : "FAIL") << "\n";
    report << "POINT_COUNT = " << points.size() << "\n";
    report << "TOTAL_DURATION_NS = " << cursor_ns << "\n";
    report << "GENERATE_S = " << generate_elapsed << "\n";
    report << "CARTESIAN_PATH_IMPOSSIBILITY = NOT_CLAIMED\n```\n\n";
    report << "이 검사는 점마다 정지하는 planning-reference UR5e 궤적을 5차 Hermite로 읽은 결과다. ";
    report << "실제 컨트롤러가 이 보간을 쓴다고 말하지 않는다. 연속 분사 효율과 실물 안전은 범위가 아니다.\n\n";
    report << "시간 하한은 구간별 나노초 올림으로 저장했고, 운동 한계는 저장 메시지를 다시 읽은 Δt로 계산했다. ";
    report << "속도 최대 절대값 " << limits_all.max_velocity << " rad/s, 가속도 최대 절대값 "
           << limits_all.max_acceleration << " rad/s^2.\n\n";
    report << "경로 이탈 " << path_deviation << ", 위치 차 최대 " << position_gap << " rad, 정지 경계 "
           << (rest_ok ? "PASS" : "FAIL") << ".\n\n";
    report << "공정 표본 " << accuracy_checked << ", 실패 " << accuracy_fail << ", 위치 최대 " << max_position
           << " m, 방향 최대 " << max_orientation << " rad.\n\n";
    report << "충돌 표본 " << samples << ", 자체 " << self_hit << ", coupon " << coupon_hit
           << ", error " << collision_error << ".\n\n";
    report << "P-022, 기하 경로, 분사 데모, TOTG 실패 기록은 이 실행에서 수정하지 않았다.\n";
  }
  {
    std::ofstream command(out_dir + "/command.txt");
    command << "ur_analytic_ik_quintic_timescale";
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
