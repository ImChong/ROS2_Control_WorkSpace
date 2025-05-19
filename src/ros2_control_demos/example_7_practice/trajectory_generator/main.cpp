#include <rclcpp/rclcpp.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

int main(int argc, char* argv[])
{
  // 初始化ROS
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("trajectory_generator");

  // 创建轨迹发布者
  auto trajectory_publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", 10);

  RCLCPP_INFO(node->get_logger(), "轨迹生成器已启动");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
