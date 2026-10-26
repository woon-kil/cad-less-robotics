// Copyright 2026 cad-less-robotics contributors

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <functional>
#include <stdexcept>
#include <string>

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

bool usableCloud(const sensor_msgs::msg::PointCloud2& msg) {
  return hasXYZ(msg) && !msg.data.empty() && msg.point_step > 0 && msg.width > 0 &&
         !msg.header.frame_id.empty();
}

class CameraNode : public rclcpp::Node {
public:
  CameraNode() : Node("camera_node") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/camera/depth/points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/vision/camera/points");
    source_ = declare_parameter<std::string>("source", "relay");
    pcd_path_ = declare_parameter<std::string>("pcd_path", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_link");
    publish_rate_ = declare_parameter<double>("publish_rate", 1.0);

    if (output_topic_.empty()) {
      throw std::invalid_argument("output_topic이 비어 있습니다.");
    }
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());

    if (source_ == "relay") {
      if (input_topic_.empty()) {
        throw std::invalid_argument("input_topic이 비어 있습니다.");
      }
      subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          input_topic_, rclcpp::SensorDataQoS(),
          std::bind(&CameraNode::onCloud, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "릴레이 %s -> %s", input_topic_.c_str(), output_topic_.c_str());
      return;
    }
    if (source_ != "pcd") {
      throw std::invalid_argument("source는 relay 또는 pcd 여야 합니다.");
    }
    if (pcd_path_.empty() || frame_id_.empty() || !(publish_rate_ > 0.0)) {
      throw std::invalid_argument("pcd 모드는 pcd_path, frame_id, publish_rate가 필요합니다.");
    }
    loadPcd();
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(period, std::bind(&CameraNode::publishCached, this));
    RCLCPP_INFO(get_logger(), "PCD 재생 %s frame=%s", pcd_path_.c_str(), frame_id_.c_str());
  }

private:
  void loadPcd() {
    pcl::PointCloud<pcl::PointXYZ> loaded;
    if (pcl::io::loadPCDFile(pcd_path_, loaded) < 0 || loaded.empty()) {
      throw std::runtime_error("PCD를 읽지 못했습니다: " + pcd_path_);
    }
    pcl::toROSMsg(loaded, cached_);
    cached_.header.frame_id = frame_id_;
  }

  void publishCached() {
    auto message = cached_;
    message.header.stamp = now();
    publisher_->publish(message);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (!usableCloud(*msg)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "xyz 필드, frame_id, 데이터가 없는 클라우드를 건너뜁니다.");
      return;
    }
    publisher_->publish(*msg);
    const std::size_t count =
        static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "릴레이 %zu 점, frame=%s", count, msg->header.frame_id.c_str());
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string source_;
  std::string pcd_path_;
  std::string frame_id_;
  double publish_rate_{1.0};
  sensor_msgs::msg::PointCloud2 cached_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("camera_node"), "%s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
