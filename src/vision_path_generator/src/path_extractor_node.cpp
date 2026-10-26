// Copyright 2026 cad-less-robotics contributors

#include "vision_path_generator/path_extractor.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <cmath>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

bool hasXYZ(const sensor_msgs::msg::PointCloud2& msg) {
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  for (const auto& field : msg.fields) {
    if (field.name == "x") {
      has_x = true;
    } else if (field.name == "y") {
      has_y = true;
    } else if (field.name == "z") {
      has_z = true;
    }
  }
  return has_x && has_y && has_z;
}

class PathExtractorNode : public rclcpp::Node {
public:
  PathExtractorNode() : Node("path_extractor_node") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/vision/camera/points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/vision/waypoints");
    filtered_topic_ = declare_parameter<std::string>("filtered_topic", "/vision/cloud_filtered");
    object_topic_ = declare_parameter<std::string>("object_topic", "/vision/cloud_object");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    require_transform_ = declare_parameter<bool>("require_transform", true);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.5);
    input_scale_ = declare_parameter<double>("input_scale", 1.0);

    vision_path_generator::ExtractParams params;
    params.voxel_leaf_size = static_cast<float>(declare_parameter<double>("voxel_leaf_size", 0.002));
    params.sor_mean_k = declare_parameter<int>("sor_mean_k", 30);
    params.sor_stddev = declare_parameter<double>("sor_stddev", 1.0);
    params.ransac_distance = declare_parameter<double>("ransac_distance", 0.004);
    params.ransac_iterations = declare_parameter<int>("ransac_iterations", 200);
    params.max_plane_inlier_ratio = declare_parameter<double>("max_plane_inlier_ratio", 0.98);
    params.min_height = declare_parameter<double>("min_height", 0.003);
    params.normal_k = declare_parameter<int>("normal_k", 20);
    params.boundary_radius = declare_parameter<double>("boundary_radius", 0.01);
    params.waypoint_spacing = declare_parameter<double>("waypoint_spacing", 0.002);
    params.max_segment_gap = declare_parameter<double>("max_segment_gap", 0.02);
    params.smooth_passes = declare_parameter<int>("smooth_passes", 1);
    params.cluster_tolerance = declare_parameter<double>("cluster_tolerance", 0.01);
    params.min_cluster_size = declare_parameter<int>("min_cluster_size", 50);
    params.min_waypoints = static_cast<std::size_t>(declare_parameter<int>("min_waypoints", 5));
    params.path_mode = declare_parameter<std::string>("path_mode", "auto");
    if (input_topic_.empty() || output_topic_.empty() || target_frame_.empty() ||
        !(tf_timeout_sec_ > 0.0) || !(input_scale_ > 0.0)) {
      throw std::invalid_argument("토픽, 프레임, TF 시간, input_scale을 확인하세요.");
    }
    extractor_ = std::make_unique<vision_path_generator::PathExtractor>(params);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(output_topic_, 10);
    filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(filtered_topic_, sensor_qos);
    object_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(object_topic_, sensor_qos);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, sensor_qos, std::bind(&PathExtractorNode::onCloud, this, std::placeholders::_1));
    worker_ = std::thread([this]() { workerLoop(); });
    RCLCPP_INFO(
        get_logger(), "경로 추출 시작. target=%s mode=%s spacing=%.3f require_tf=%s",
        target_frame_.c_str(), params.path_mode.c_str(), params.waypoint_spacing,
        require_transform_ ? "true" : "false");
  }

  ~PathExtractorNode() override {
    requestStop();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void requestStop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    cv_.notify_all();
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (busy_ || stop_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "이전 점군을 처리 중이라 이번 프레임을 버립니다.");
      return;
    }
    pending_ = msg;
    cv_.notify_one();
  }

  void workerLoop() {
    while (rclcpp::ok()) {
      sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
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
        process(msg);
      } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "점군 처리 예외: %s", ex.what());
      }
      std::lock_guard<std::mutex> lock(mutex_);
      busy_ = false;
    }
  }

  void process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    if (!msg || msg->data.empty() || msg->header.frame_id.empty() || !hasXYZ(*msg)) {
      RCLCPP_WARN(get_logger(), "xyz, frame_id, 데이터 중 하나가 없는 점군을 건너뜁니다.");
      return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string output_frame = msg->header.frame_id;
    if (std::abs(input_scale_ - 1.0) > 1e-12) {
      pcl::fromROSMsg(*msg, *cloud);
      for (auto& point : cloud->points) {
        point.x = static_cast<float>(point.x * input_scale_);
        point.y = static_cast<float>(point.y * input_scale_);
        point.z = static_cast<float>(point.z * input_scale_);
      }
      sensor_msgs::msg::PointCloud2 scaled;
      pcl::toROSMsg(*cloud, scaled);
      scaled.header = msg->header;
      if (!transformCloud(scaled, output_frame, cloud)) {
        return;
      }
    } else if (!transformCloud(*msg, output_frame, cloud)) {
      return;
    }

    const auto result = extractor_->extract(cloud);
    std_msgs::msg::Header header;
    header.stamp = msg->header.stamp;
    header.frame_id = output_frame;
    publishCloud(filtered_pub_, result.filtered, header);
    publishCloud(object_pub_, result.object, header);
    if (!result.error.empty()) {
      RCLCPP_ERROR(
          get_logger(), "%s (in=%zu voxel=%zu sor=%zu plane=%zu object=%zu)", result.error.c_str(),
          result.input_points, result.voxel_points, result.sor_points, result.plane_inliers,
          result.object_points);
      return;
    }

    geometry_msgs::msg::PoseArray poses;
    poses.header = header;
    poses.poses.reserve(result.waypoints.size());
    for (const auto& waypoint : result.waypoints) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = waypoint.position.x;
      pose.position.y = waypoint.position.y;
      pose.position.z = waypoint.position.z;
      pose.orientation.x = waypoint.orientation.x;
      pose.orientation.y = waypoint.orientation.y;
      pose.orientation.z = waypoint.orientation.z;
      pose.orientation.w = waypoint.orientation.w;
      poses.poses.push_back(pose);
    }
    pose_pub_->publish(poses);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "waypoints=%zu method=%s frame=%s", poses.poses.size(),
        result.method.c_str(), output_frame.c_str());
  }

  bool transformCloud(
      const sensor_msgs::msg::PointCloud2& input,
      std::string& output_frame,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& output) {
    if (input.header.frame_id == target_frame_) {
      pcl::fromROSMsg(input, *output);
      output_frame = target_frame_;
      return true;
    }
    if (!require_transform_) {
      pcl::fromROSMsg(input, *output);
      output_frame = input.header.frame_id;
      RCLCPP_WARN_ONCE(
          get_logger(), "TF 없이 %s 프레임 포즈를 발행합니다. 로봇 실행 전에는 계획 프레임과 연결해야 합니다.",
          output_frame.c_str());
      return true;
    }
    try {
      const auto transform = tf_buffer_->lookupTransform(
          target_frame_, input.header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(tf_timeout_sec_));
      sensor_msgs::msg::PointCloud2 transformed;
      tf2::doTransform(input, transformed, transform);
      pcl::fromROSMsg(transformed, *output);
      output_frame = target_frame_;
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(
          get_logger(), "TF %s -> %s 실패: %s. 포즈를 발행하지 않습니다.", input.header.frame_id.c_str(),
          target_frame_.c_str(), ex.what());
      return false;
    }
  }

  static void publishCloud(
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      const std_msgs::msg::Header& header) {
    if (!publisher || !cloud || cloud->empty()) {
      return;
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*cloud, message);
    message.header = header;
    publisher->publish(message);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string filtered_topic_;
  std::string object_topic_;
  std::string target_frame_;
  bool require_transform_{true};
  double tf_timeout_sec_{0.5};
  double input_scale_{1.0};
  std::unique_ptr<vision_path_generator::PathExtractor> extractor_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_pub_;
  std::mutex mutex_;
  std::condition_variable cv_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_;
  bool busy_{false};
  bool stop_{false};
  std::thread worker_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<PathExtractorNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    rclcpp::on_shutdown([node]() { node->requestStop(); });
    executor.spin();
    node->requestStop();
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("path_extractor_node"), "%s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
