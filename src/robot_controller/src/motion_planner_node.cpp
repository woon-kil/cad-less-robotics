// Copyright 2026 cad-less-robotics contributors

#include "robot_controller/waypoint_prep.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace robot_controller {
namespace {

Pose fromMsg(const geometry_msgs::msg::Pose& msg) {
  return Pose{msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y,
              msg.orientation.z, msg.orientation.w};
}

geometry_msgs::msg::Pose toMsg(const Pose& pose) {
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.x;
  msg.position.y = pose.y;
  msg.position.z = pose.z;
  msg.orientation.x = pose.qx;
  msg.orientation.y = pose.qy;
  msg.orientation.z = pose.qz;
  msg.orientation.w = pose.qw;
  return msg;
}

double jointDelta(const trajectory_msgs::msg::JointTrajectoryPoint& a,
                  const trajectory_msgs::msg::JointTrajectoryPoint& b) {
  if (a.positions.size() != b.positions.size() || a.positions.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double max_delta = 0.0;
  for (std::size_t index = 0; index < a.positions.size(); ++index) {
    max_delta = std::max(max_delta, std::abs(a.positions[index] - b.positions[index]));
  }
  return max_delta;
}

bool timestampsIncrease(const trajectory_msgs::msg::JointTrajectory& trajectory) {
  if (trajectory.points.size() < 2) {
    return false;
  }
  for (std::size_t index = 1; index < trajectory.points.size(); ++index) {
    const rclcpp::Duration previous(trajectory.points[index - 1].time_from_start);
    const rclcpp::Duration current(trajectory.points[index].time_from_start);
    if (current <= previous) {
      return false;
    }
  }
  return true;
}

bool withinJointVelocity(const trajectory_msgs::msg::JointTrajectory& trajectory, double max_velocity) {
  if (!timestampsIncrease(trajectory) || !(max_velocity > 0.0)) {
    return false;
  }
  for (std::size_t index = 1; index < trajectory.points.size(); ++index) {
    const double dt = (rclcpp::Duration(trajectory.points[index].time_from_start) -
                       rclcpp::Duration(trajectory.points[index - 1].time_from_start))
                          .seconds();
    if (!(dt > 1e-4)) {
      return false;
    }
    if (jointDelta(trajectory.points[index - 1], trajectory.points[index]) / dt > max_velocity * 1.001) {
      return false;
    }
  }
  return true;
}

void retime(trajectory_msgs::msg::JointTrajectory& trajectory, double max_velocity) {
  if (trajectory.points.empty()) {
    return;
  }
  trajectory.points.front().time_from_start = rclcpp::Duration(0, 0);
  for (std::size_t index = 1; index < trajectory.points.size(); ++index) {
    const double delta = jointDelta(trajectory.points[index - 1], trajectory.points[index]);
    const double dt = std::max(delta / max_velocity, 0.05);
    trajectory.points[index].time_from_start =
        trajectory.points[index - 1].time_from_start + rclcpp::Duration::from_seconds(dt);
  }
}

class MotionPlannerNode : public rclcpp::Node {
public:
  MotionPlannerNode() : Node("motion_planner_node") {
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/vision/waypoints");
    planning_group_ = declare_parameter<std::string>("planning_group", "arm_group");
    end_effector_link_ = declare_parameter<std::string>("end_effector_link", "Arm5_Link");
    planning_frame_ = declare_parameter<std::string>("planning_frame", "base_link");
    eef_step_ = declare_parameter<double>("eef_step", 0.002);
    jump_threshold_ = declare_parameter<double>("jump_threshold", 1.0);
    avoid_collisions_ = declare_parameter<bool>("avoid_collisions", true);
    min_fraction_ = declare_parameter<double>("min_fraction", 0.95);
    velocity_scaling_ = declare_parameter<double>("velocity_scaling", 0.15);
    acceleration_scaling_ = declare_parameter<double>("acceleration_scaling", 0.15);
    execute_trajectory_ = declare_parameter<bool>("execute_trajectory", false);
    approach_offset_m_ = declare_parameter<double>("approach_offset_m", 0.02);
    retreat_offset_m_ = declare_parameter<double>("retreat_offset_m", 0.02);
    max_waypoints_ = static_cast<std::size_t>(declare_parameter<int>("max_waypoints", 400));
    min_pose_separation_ = declare_parameter<double>("min_pose_separation", 0.0005);
    max_reach_m_ = declare_parameter<double>("max_reach_m", 1.0);
    max_pose_age_sec_ = declare_parameter<double>("max_pose_age_sec", 3.0);
    goal_position_tolerance_ = declare_parameter<double>("goal_position_tolerance", 0.001);
    goal_orientation_tolerance_ = declare_parameter<double>("goal_orientation_tolerance", 0.05);
    planning_time_ = declare_parameter<double>("planning_time", 5.0);
    server_timeout_sec_ = declare_parameter<double>("server_timeout_sec", 10.0);
    fallback_joint_velocity_ = declare_parameter<double>("fallback_joint_velocity", 0.4);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.5);
    validateParams();
    if (jump_threshold_ == 0.0) {
      RCLCPP_WARN(
          get_logger(),
          "jump_threshold가 0입니다. Cartesian IK 점프 검사를 끄면 실기에서 큰 관절 이동이 생길 수 있습니다.");
    }
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  ~MotionPlannerNode() override {
    requestStop();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  bool initialize() {
    try {
      auto group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), planning_group_, tf_buffer_,
          rclcpp::Duration::from_seconds(server_timeout_sec_));
      std::lock_guard<std::mutex> lock(group_mutex_);
      move_group_ = group;
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface 초기화 실패: %s", ex.what());
      return false;
    }

    auto group = groupCopy();
    if (!planning_frame_.empty()) {
      group->setPoseReferenceFrame(planning_frame_);
    }
    active_frame_ = group->getPoseReferenceFrame();
    RCLCPP_INFO(
        get_logger(), "MoveIt planning_frame=%s pose_reference=%s", group->getPlanningFrame().c_str(),
        active_frame_.c_str());
    if (!end_effector_link_.empty() && !group->setEndEffectorLink(end_effector_link_)) {
      RCLCPP_WARN(
          get_logger(), "엔드이펙터 링크 '%s' 설정 실패. 그룹 기본 팁을 사용합니다.",
          end_effector_link_.c_str());
    }
    RCLCPP_INFO(get_logger(), "엔드이펙터 링크=%s", group->getEndEffectorLink().c_str());
    group->setMaxVelocityScalingFactor(velocity_scaling_);
    group->setMaxAccelerationScalingFactor(acceleration_scaling_);
    group->setGoalPositionTolerance(goal_position_tolerance_);
    group->setGoalOrientationTolerance(goal_orientation_tolerance_);
    group->setPlanningTime(planning_time_);

    trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("~/planned_trajectory", 10);
    worker_ = std::thread([this]() { workerLoop(); });
    subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
        pose_topic_, 10, std::bind(&MotionPlannerNode::onPoses, this, std::placeholders::_1));
    RCLCPP_INFO(
        get_logger(), "모션 플래너 대기. execute_trajectory=%s group=%s",
        execute_trajectory_ ? "true" : "false", planning_group_.c_str());
    return true;
  }

  void requestStop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    cv_.notify_all();
    auto group = groupCopy();
    if (group) {
      try {
        group->stop();
      } catch (const std::exception& ex) {
        RCLCPP_WARN(get_logger(), "정지 요청 실패: %s", ex.what());
      }
    }
  }

private:
  void validateParams() {
    if (pose_topic_.empty() || planning_group_.empty()) {
      throw std::invalid_argument("pose_topic과 planning_group은 비울 수 없습니다.");
    }
    if (!(eef_step_ > 0.0) || !(server_timeout_sec_ > 0.0) || !(fallback_joint_velocity_ > 0.0) ||
        !(tf_timeout_sec_ > 0.0) || !(max_reach_m_ > 0.0) || !(planning_time_ > 0.0)) {
      throw std::invalid_argument("양수 파라미터가 0 이하입니다.");
    }
    if (jump_threshold_ < 0.0 || approach_offset_m_ < 0.0 || retreat_offset_m_ < 0.0 ||
        max_pose_age_sec_ < 0.0 || min_pose_separation_ < 0.0) {
      throw std::invalid_argument("음수일 수 없는 파라미터가 음수입니다.");
    }
    if (!(min_fraction_ > 0.0) || min_fraction_ > 1.0 || !(velocity_scaling_ > 0.0) ||
        velocity_scaling_ > 1.0 || !(acceleration_scaling_ > 0.0) || acceleration_scaling_ > 1.0) {
      throw std::invalid_argument("분율 또는 속도 스케일이 (0, 1] 밖입니다.");
    }
    if (max_waypoints_ < 2) {
      throw std::invalid_argument("max_waypoints는 2 이상이어야 합니다.");
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> groupCopy() {
    std::lock_guard<std::mutex> lock(group_mutex_);
    return move_group_;
  }

  void onPoses(const geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (busy_ || stop_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "플래너가 바빠 이번 PoseArray를 버립니다.");
      return;
    }
    pending_ = msg;
    cv_.notify_one();
  }

  void workerLoop() {
    while (rclcpp::ok()) {
      geometry_msgs::msg::PoseArray::ConstSharedPtr msg;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return stop_ || pending_ || !rclcpp::ok(); });
        if (stop_ || !rclcpp::ok()) {
          return;
        }
        msg = pending_;
        pending_.reset();
        busy_ = true;
      }
      try {
        planAndMaybeExecute(msg);
      } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "계획 예외: %s", ex.what());
      }
      std::lock_guard<std::mutex> lock(mutex_);
      busy_ = false;
    }
  }

  void planAndMaybeExecute(const geometry_msgs::msg::PoseArray::ConstSharedPtr& msg) {
    if (!msg || msg->poses.empty() || msg->header.frame_id.empty()) {
      RCLCPP_ERROR(get_logger(), "비어 있거나 frame_id가 없는 PoseArray는 실행하지 않습니다.");
      return;
    }
    if (!freshEnough(*msg)) {
      return;
    }
    std::vector<geometry_msgs::msg::Pose> transformed;
    if (!transformPoses(*msg, transformed)) {
      return;
    }
    std::vector<Pose> poses;
    poses.reserve(transformed.size());
    for (const auto& pose : transformed) {
      poses.push_back(fromMsg(pose));
    }
    if (!normalizeOrientations(poses)) {
      RCLCPP_ERROR(get_logger(), "쿼터니언을 정규화할 수 없어 실행하지 않습니다.");
      return;
    }
    alignQuaternionHemisphere(poses);
    poses = dropClosePoses(poses, min_pose_separation_);
    poses = limitWaypointCount(poses, max_waypoints_);
    poses = withApproachRetreat(poses, approach_offset_m_, retreat_offset_m_);
    alignQuaternionHemisphere(poses);
    const std::string error = pathError(poses, max_reach_m_);
    if (!error.empty()) {
      RCLCPP_ERROR(get_logger(), "%s 실행하지 않습니다.", error.c_str());
      return;
    }

    auto group = groupCopy();
    if (!group) {
      RCLCPP_ERROR(get_logger(), "MoveGroup이 없습니다.");
      return;
    }
    if (!group->getCurrentState(1.0)) {
      RCLCPP_ERROR(get_logger(), "현재 관절 상태를 가져오지 못했습니다. joint_states를 확인하세요.");
      return;
    }
    group->setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(poses.size());
    for (const auto& pose : poses) {
      waypoints.push_back(toMsg(pose));
    }
    moveit_msgs::msg::RobotTrajectory trajectory;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    const double fraction = group->computeCartesianPath(
        waypoints, eef_step_, jump_threshold_, trajectory, avoid_collisions_, &error_code);
    if (!std::isfinite(fraction) || fraction < 0.0) {
      RCLCPP_ERROR(get_logger(), "Cartesian 계획 실패 fraction=%.3f code=%d", fraction, error_code.val);
      return;
    }
    RCLCPP_INFO(get_logger(), "Cartesian 경로 %.1f%% (%zu waypoints)", fraction * 100.0, waypoints.size());
    if (trajectory.joint_trajectory.points.size() < 2) {
      RCLCPP_ERROR(get_logger(), "궤적 점이 2개 미만이라 실행하지 않습니다.");
      return;
    }
    if (!prepareTiming(trajectory.joint_trajectory)) {
      return;
    }
    trajectory_pub_->publish(trajectory.joint_trajectory);

    if (fraction + 1e-3 < 1.0 || fraction < min_fraction_) {
      RCLCPP_ERROR(
          get_logger(),
          "전체 웨이포인트를 포함하지 않는 부분 경로입니다 (%.1f%%, 하한 %.1f%%). 실행하지 않습니다.",
          fraction * 100.0, min_fraction_ * 100.0);
      return;
    }
    if (!execute_trajectory_) {
      RCLCPP_INFO(get_logger(), "execute_trajectory가 false라 계획만 발행했습니다.");
      return;
    }
    if (!rclcpp::ok()) {
      return;
    }
    const auto code = group->execute(trajectory);
    if (!code) {
      RCLCPP_ERROR(get_logger(), "궤적 실행 실패 code=%d", code.val);
      return;
    }
    RCLCPP_INFO(get_logger(), "궤적 실행을 마쳤습니다.");
  }

  bool freshEnough(const geometry_msgs::msg::PoseArray& msg) const {
    if (!(max_pose_age_sec_ > 0.0)) {
      return true;
    }
    const rclcpp::Time stamp(msg.header.stamp);
    if (stamp.nanoseconds() == 0) {
      RCLCPP_ERROR(get_logger(), "포즈 스탬프가 없어 신선도를 확인할 수 없습니다. 실행하지 않습니다.");
      return false;
    }
    const double age = (now() - stamp).seconds();
    if (age < -0.5) {
      RCLCPP_WARN(get_logger(), "포즈 스탬프가 현재보다 %.3f초 앞섭니다.", -age);
    } else if (age > max_pose_age_sec_) {
      RCLCPP_ERROR(
          get_logger(), "포즈가 %.3f초 전 데이터라 실행하지 않습니다 (한계 %.3f초).", age, max_pose_age_sec_);
      return false;
    }
    return true;
  }

  bool transformPoses(
      const geometry_msgs::msg::PoseArray& msg, std::vector<geometry_msgs::msg::Pose>& output) {
    if (msg.header.frame_id == active_frame_) {
      output = msg.poses;
      return true;
    }
    try {
      const auto transform = tf_buffer_->lookupTransform(
          active_frame_, msg.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_sec_));
      output.reserve(msg.poses.size());
      for (const auto& pose : msg.poses) {
        geometry_msgs::msg::PoseStamped in;
        geometry_msgs::msg::PoseStamped out;
        in.header = msg.header;
        in.pose = pose;
        tf2::doTransform(in, out, transform);
        output.push_back(out.pose);
      }
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(
          get_logger(), "TF %s -> %s 실패: %s. 실행하지 않습니다.", msg.header.frame_id.c_str(),
          active_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool prepareTiming(trajectory_msgs::msg::JointTrajectory& trajectory) {
    if (trajectory.points.front().positions.empty()) {
      RCLCPP_ERROR(get_logger(), "궤적에 관절 위치가 없습니다.");
      return false;
    }
    if (!withinJointVelocity(trajectory, fallback_joint_velocity_)) {
      RCLCPP_WARN(
          get_logger(), "궤적 시간이 관절 속도 상한 %.3f rad/s를 넘거나 비어 있어 다시 맞춥니다.",
          fallback_joint_velocity_);
      retime(trajectory, fallback_joint_velocity_);
    }
    if (!withinJointVelocity(trajectory, fallback_joint_velocity_)) {
      RCLCPP_ERROR(get_logger(), "궤적 시간을 속도 상한 안으로 맞추지 못했습니다. 실행하지 않습니다.");
      return false;
    }
    return true;
  }

  std::string pose_topic_;
  std::string planning_group_;
  std::string end_effector_link_;
  std::string planning_frame_;
  std::string active_frame_;
  double eef_step_{0.002};
  double jump_threshold_{1.0};
  bool avoid_collisions_{true};
  double min_fraction_{0.95};
  double velocity_scaling_{0.15};
  double acceleration_scaling_{0.15};
  bool execute_trajectory_{false};
  double approach_offset_m_{0.02};
  double retreat_offset_m_{0.02};
  std::size_t max_waypoints_{400};
  double min_pose_separation_{0.0005};
  double max_reach_m_{1.0};
  double max_pose_age_sec_{3.0};
  double goal_position_tolerance_{0.001};
  double goal_orientation_tolerance_{0.05};
  double planning_time_{5.0};
  double server_timeout_sec_{10.0};
  double fallback_joint_velocity_{0.4};
  double tf_timeout_sec_{0.5};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  std::mutex mutex_;
  std::mutex group_mutex_;
  std::condition_variable cv_;
  geometry_msgs::msg::PoseArray::ConstSharedPtr pending_;
  bool busy_{false};
  bool stop_{false};
  std::thread worker_;
};

}  // namespace
}  // namespace robot_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<robot_controller::MotionPlannerNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    rclcpp::on_shutdown([node]() { node->requestStop(); });
    std::thread spinner([&executor]() { executor.spin(); });
    if (!node->initialize()) {
      exit_code = 1;
      rclcpp::shutdown();
    }
    spinner.join();
    node->requestStop();
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("motion_planner_node"), "%s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
