// 저장된 관절쌍의 bounded-revolute 중간점 FK 오차. IK는 호출하지 않는다.

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

namespace
{
std::string read_file(const std::string & path)
{
  std::ifstream input(path);
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3)
  {
    std::cerr << "usage: p022_fk_midpoint urdf srdf\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  auto urdf = urdf::parseURDF(read_file(argv[1]));
  auto srdf = std::make_shared<srdf::Model>();
  if (!urdf || !srdf->initString(*urdf, read_file(argv[2])))
  {
    return 3;
  }
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  const auto * group = model->getJointModelGroup("ur_manipulator");
  const auto * tool = model->getLinkModel("tool0");
  std::string name;
  while (std::cin >> name)
  {
    std::vector<double> start(6);
    std::vector<double> end(6);
    for (double & value : start)
    {
      std::cin >> value;
    }
    for (double & value : end)
    {
      std::cin >> value;
    }
    moveit::core::RobotState start_state(model);
    moveit::core::RobotState end_state(model);
    start_state.setToDefaultValues();
    end_state.setToDefaultValues();
    start_state.setJointGroupPositions(group, start);
    end_state.setJointGroupPositions(group, end);
    start_state.update();
    end_state.update();
    moveit::core::RobotState mid_state(model);
    start_state.interpolate(end_state, 0.5, mid_state);
    mid_state.update();
    const Eigen::Isometry3d start_pose = start_state.getGlobalLinkTransform(tool);
    const Eigen::Isometry3d end_pose = end_state.getGlobalLinkTransform(tool);
    const Eigen::Isometry3d fk_mid = mid_state.getGlobalLinkTransform(tool);
    Eigen::Isometry3d cartesian_mid(
      Eigen::Quaterniond(start_pose.linear()).slerp(0.5, Eigen::Quaterniond(end_pose.linear())));
    cartesian_mid.translation() = 0.5 * (start_pose.translation() + end_pose.translation());
    const double position = (cartesian_mid.translation() - fk_mid.translation()).norm();
    const double orientation =
      Eigen::Quaterniond(cartesian_mid.linear()).angularDistance(Eigen::Quaterniond(fk_mid.linear()));
    double joint_delta = 0;
    int joint_index = 0;
    for (int index = 0; index < 6; ++index)
    {
      const double delta = std::abs(start[index] - end[index]);
      if (delta > joint_delta)
      {
        joint_delta = delta;
        joint_index = index;
      }
    }
    std::cout << name << " joint_delta " << joint_delta << " joint_index " << joint_index << " position_error "
              << position << " orientation_error " << orientation << " fk_start " << start_pose.translation().x() << " "
              << start_pose.translation().y() << " " << start_pose.translation().z() << " fk_end "
              << end_pose.translation().x() << " " << end_pose.translation().y() << " " << end_pose.translation().z()
              << "\n";
  }
  rclcpp::shutdown();
  return 0;
}
