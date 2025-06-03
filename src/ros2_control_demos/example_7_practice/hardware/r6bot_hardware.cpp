#include "r6bot_hardware.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <iostream>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// helper 函数
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void print_vector_double(const std::vector<double> & vec)
{
  std::cout << "[ ";
  for (const auto & item : vec)
  {
    std::cout << item << " ";
  }
  std::cout << "]" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 命名空间
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace r6bot_hardware_namespace
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 初始化
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
hardware_interface::CallbackReturn R6BotHardware::on_init(const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  std::cout << "===== 初始化开始 =====" << std::endl;

  // 获取硬件信息
  hardware_info_ = info;

  // 初始化状态
  joint_position_states_.assign(6, 0);
  joint_velocities_states_.assign(6, 0);
  ft_states_.assign(6, 0);

  // 初始化命令
  joint_position_command_.assign(6, 0);
  joint_velocities_command_.assign(6, 0);
  ft_command_.assign(6, 0);

  // 打印状态
  std::cout << "joint_position_states_: ";
  print_vector_double(joint_position_states_);
  std::cout << "joint_velocities_states_: ";
  print_vector_double(joint_velocities_states_);
  std::cout << "ft_states_: ";
  print_vector_double(ft_states_);

  // 打印命令
  std::cout << "joint_position_command_: ";
  print_vector_double(joint_position_command_);
  std::cout << "joint_velocities_command_: ";
  print_vector_double(joint_velocities_command_);
  std::cout << "ft_command_: ";
  print_vector_double(ft_command_);

  // 打印关节接口
  for (const auto & joint : hardware_info_.joints)
  {
    // 打印状态接口
    std::cout << joint.name << ":" << std::endl;
    std::cout << "\t" << "state interfaces: " << std::endl;
    for (const auto & interface : joint.state_interfaces)
    {
      std::cout << "\t\t" << interface.name << std::endl;
    }
    // 打印命令接口
    std::cout << "\t" << "command interfaces: " << std::endl;
    for (const auto & interface : joint.command_interfaces)
    {
      std::cout << "\t\t" << interface.name << std::endl;
    }
  }

  // 获取关节接口
  for (const auto & joint : hardware_info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  // 打印位置接口
  std::cout << "joint_interfaces[\"position\"]";
  for (const auto & joint : joint_interfaces["position"])
  {
    std::cout << joint << " ";
  }
  std::cout << std::endl;

  // 打印速度接口
  std::cout << "joint_interfaces[\"velocity\"]";
  for (const auto & joint : joint_interfaces["velocity"])
  {
    std::cout << joint << " ";
  }
  std::cout << std::endl;

  std::cout << "===== 初始化完成 =====" << std::endl;
  return CallbackReturn::SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 导出状态接口
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<hardware_interface::StateInterface> R6BotHardware::export_state_interfaces()
{
  std::cout << "===== 导出状态接口开始 =====" << std::endl;
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // 导出位置接口
  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_states_[ind++]);
  }

  // 导出速度接口
  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_states_[ind++]);
  }

  // 导出力矩接口
  state_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_states_[0]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_states_[1]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_states_[2]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_states_[3]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_states_[4]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_states_[5]);

  std::cout << "===== 导出状态接口完成 =====" << std::endl;
  return state_interfaces;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 导出命令接口
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<hardware_interface::CommandInterface> R6BotHardware::export_command_interfaces()
{
  std::cout << "===== 导出命令接口开始 =====" << std::endl;
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // 导出位置接口
  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  // 导出速度接口
  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  // 导出力矩接口
  command_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_command_[0]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_command_[1]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_command_[2]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_command_[3]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_command_[4]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_command_[5]);

  std::cout << "===== 导出命令接口完成 =====" << std::endl;
  return command_interfaces;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 读取
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
hardware_interface::return_type R6BotHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::cout << "===== 读取开始 =====" << std::endl;
  std::cout << "time: " << std::fixed << std::setprecision(3) << time.seconds() << std::endl;
  std::cout << "period: " << std::fixed << std::setprecision(3) << period.seconds() << std::endl;

  // 读取关节位置命令
  // for (auto i = 0ul; i < joint_position_command_.size(); i++)
  // {
  //   joint_position_states_[i] = joint_position_command_[i];
  // }

  // 读取关节速度命令
  for (auto i = 0ul; i < joint_velocities_command_.size(); i++)
  {
    joint_velocities_states_[i] = joint_velocities_command_[i];
    joint_position_states_[i] += joint_velocities_command_[i] * period.seconds();
  }

  // 读取力矩命令
  for (auto i = 0ul; i < ft_command_.size(); i++)
  {
    ft_states_[i] = ft_command_[i];
  }

  // 打印状态
  std::cout << "joint_position_states_: ";
  print_vector_double(joint_position_states_);
  std::cout << "joint_velocities_states_: ";
  print_vector_double(joint_velocities_states_);
  std::cout << "ft_states_: ";
  print_vector_double(ft_states_);

  std::cout << "===== 读取完成 =====" << std::endl;
  return hardware_interface::return_type::OK;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 写入
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
hardware_interface::return_type R6BotHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::cout << "===== 写入开始 =====" << std::endl;
  std::cout << "time: " << std::fixed << std::setprecision(3) << time.seconds() << std::endl;
  std::cout << "period: " << std::fixed << std::setprecision(3) << period.seconds() << std::endl;

  std::cout << "joint_position_command_: ";
  print_vector_double(joint_position_command_);
  std::cout << "joint_velocities_command_: ";
  print_vector_double(joint_velocities_command_);
  std::cout << "ft_command_: ";
  print_vector_double(ft_command_);

  std::cout << "===== 写入完成 =====" << std::endl;
  return hardware_interface::return_type::OK;
}

}   // namespace r6bot_hardware_namespace

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 插件注册
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(r6bot_hardware_namespace::R6BotHardware, hardware_interface::SystemInterface)
