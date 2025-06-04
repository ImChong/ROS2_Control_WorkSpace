#include <rclcpp/rclcpp.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

int main(int argc, char* argv[])
{
  // 初始化ROS //////////////////////////////////////////////////////////////////////////////////////////////////////////
  rclcpp::init(argc, argv); // 初始化ROS
  auto node = std::make_shared<rclcpp::Node>("trajectory_generator"); // 创建节点
  auto trajectory_publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", 10); // 创建轨迹发布者

  // 获取参数 ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  auto robot_param = rclcpp::Parameter(); // 声明参数
  node->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);  // 声明参数类型
  node->get_parameter("robot_description", robot_param); // 获取参数
  auto robot_description = robot_param.as_string(); // 转换为字符串

  // 创建机器人模型 //////////////////////////////////////////////////////////////////////////////////////////////////////
  KDL::Tree robot_tree; // 创建机器人树
  KDL::Chain chain; // 创建机器人链
  kdl_parser::treeFromString(robot_description, robot_tree); // 从字符串创建机器人树
  robot_tree.getChain("base_link", "tool0", chain); // 获取机器人链

  // 创建IK求解器/////////////////////////////////////////////////////////////////////////////////////////////////////////
  auto joint_positions = KDL::JntArray(chain.getNrOfJoints());
  auto joint_velocities = KDL::JntArray(chain.getNrOfJoints());
  auto twist = KDL::Twist();
  auto ik_vel_solver = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain, 0.0000001);

  // 创建轨迹消息/////////////////////////////////////////////////////////////////////////////////////////////////////////
  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = node->now();
  for (size_t i = 0; i < chain.getNrOfSegments(); i++)
  {
    auto joint = chain.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::Fixed)
    {
      trajectory_msg.joint_names.push_back(joint.getName());
    }
  }

  trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
  trajectory_point_msg.positions.resize(chain.getNrOfJoints());
  trajectory_point_msg.velocities.resize(chain.getNrOfJoints());

  // 创建轨迹///////////////////////////////////////////////////////////////////////////////////////////////////////////
  double total_time = 3.0;
  int trajectory_len = 200;
  int loop_rate = trajectory_len / total_time;
  double dt = 1.0 / loop_rate;

  // 生成轨迹///////////////////////////////////////////////////////////////////////////////////////////////////////////
  RCLCPP_INFO(node->get_logger(), "===== 轨迹生成开始 =====");
  for (int i = 0; i < trajectory_len; i++)
  {
    double t = i;
    twist.vel.x(2 * 0.3 * cos(2 * M_PI * t / trajectory_len));  // x方向的角速度
    twist.vel.y(-0.3 * sin(2 * M_PI * t / trajectory_len));  // y方向的角速度

    ik_vel_solver->CartToJnt(joint_positions, twist, joint_velocities);  // 计算关节角速度

    RCLCPP_INFO(node->get_logger(), "joint_positions: ");
    for (unsigned int j = 0; j < joint_positions.rows(); ++j) {
      RCLCPP_INFO(node->get_logger(), "%f ", joint_positions(j));
    }
    RCLCPP_INFO(node->get_logger(), "\n");

    RCLCPP_INFO(node->get_logger(), "joint_velocities: ");
    for (unsigned int j = 0; j < joint_velocities.rows(); ++j) {
      RCLCPP_INFO(node->get_logger(), "%f ", joint_velocities(j));
    }
    RCLCPP_INFO(node->get_logger(), "\n");

    // 填充轨迹消息
    for (unsigned int j = 0; j < joint_positions.rows(); ++j) {
      trajectory_point_msg.positions[j] = joint_positions(j);
      trajectory_point_msg.velocities[j] = joint_velocities(j);
    }

    // 更新关节位置
    for (unsigned int j = 0; j < joint_positions.rows(); ++j) {
      joint_positions(j) += joint_velocities(j) * dt;
    }

    // 填充时间信息
    trajectory_point_msg.time_from_start.sec = i / loop_rate;
    trajectory_point_msg.time_from_start.nanosec = static_cast<int>(1E9 / loop_rate * static_cast<double>(t - loop_rate * (i / loop_rate)));
    trajectory_msg.points.push_back(trajectory_point_msg);
    RCLCPP_INFO(node->get_logger(), "time_from_start: %d.%d\n", trajectory_point_msg.time_from_start.sec, trajectory_point_msg.time_from_start.nanosec);
  }

  trajectory_publisher->publish(trajectory_msg);  // 发布轨迹消息
  RCLCPP_INFO(node->get_logger(), "===== 轨迹生成完成 =====");

  // while (rclcpp::ok()){}
  return 0;
}
