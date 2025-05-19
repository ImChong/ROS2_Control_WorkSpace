#include "rrbot.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rrbot_hardware
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 打印向量
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void print_vector_double(const std::vector<double> & vec)
{
  std::cout << "[ ";
  for (auto val : vec) {
    std::cout << val << " ";
  }
  std::cout << "]" << std::endl;
}

void print_vector_component_info(const std::vector<hardware_interface::ComponentInfo> & vec)
{
  std::cout << "[ ";
  for (auto val : vec) {
    std::cout << val.name << " ";
  }
  std::cout << "]" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 初始化硬件接口
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
hardware_interface::CallbackReturn RRBotHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
      return hardware_interface::CallbackReturn::ERROR;
  }

  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("RRBot 硬件消息"));  // 初始化日志
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());  // 初始化时钟

  std::cout << "===== 初始化硬件接口 =====" << std::endl;
  // // 打印所有info
  // std::cout << "info_.name[名称]: " << info_.name << std::endl;
  // std::cout << "info_.type[类型]: " << info_.type << std::endl;
  // std::cout << "info_.hardware_class_type[硬件类型]: " << info_.hardware_class_type << std::endl;
  // std::cout << "info_.joints.size()[关节数量]: " << info_.joints.size() << std::endl;

  std::cout << "info_.joints[关节列表]: ";
  print_vector_component_info(info_.joints);

  // // 打印所有joints信息
  // std::cout << "info_.joints[关节列表]: " << std::endl;
  // for (const auto & joint : info_.joints)
  // {
  //   std::cout << "\tjoint.name[关节名称]: " << joint.name << std::endl;
  //   std::cout << "\tjoint.type[关节类型]: " << joint.type << std::endl;
  //   std::cout << "\tjoint.command_interfaces[命令接口]: " << std::endl;
  //   for (const auto & command_interface : joint.command_interfaces)
  //   {
  //     std::cout << "\t\tcommand_interface.name[接口名称]: " << command_interface.name << std::endl;
  //     std::cout << "\t\tcommand_interface.data_type[接口数据类型]: " << command_interface.data_type << std::endl;
  //     std::cout << "\t\tcommand_interface.size[接口大小]: " << command_interface.size << std::endl;
  //   }
  //   std::cout << "\tjoint.state_interfaces[状态接口]: " << std::endl;
  //   for (const auto & state_interface : joint.state_interfaces)
  //   {
  //     std::cout << "\t\tstate_interface.name[接口名称]: " << state_interface.name << std::endl;
  //     std::cout << "\t\tstate_interface.data_type[接口数据类型]: " << state_interface.data_type << std::endl;
  //     std::cout << "\t\tstate_interface.size[接口大小]: " << state_interface.size << std::endl;
  //   }
  // }

  // 打印所有参数
  // std::cout << "info_.hardware_parameters[硬件参数]: " << std::endl;
  // for (const auto & param : info_.hardware_parameters)
  // {
  //   std::cout << "\t" << param.first << ": " << param.second << std::endl;
  // }
  hw_start_sec_ = stod(info_.hardware_parameters["hw_start_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["hw_stop_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["hw_slowdown"]);

  std::cout << "hw_start_sec_[启动时间]: " << hw_start_sec_ << std::endl;
  std::cout << "hw_stop_sec_[停止时间]: " << hw_stop_sec_ << std::endl;
  std::cout << "hw_slowdown_[减速时间]: " << hw_slowdown_ << std::endl;

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // std::cout << "hw_states_.size()[状态接口数量]: " << hw_states_.size() << std::endl;
  // std::cout << "hw_states_[状态接口]: ";
  // print_vector_double(hw_states_);

  // std::cout << "hw_commands_.size()[命令接口数量]: " << hw_commands_.size() << std::endl;
  // std::cout << "hw_commands_[命令接口]: ";
  // print_vector_double(hw_commands_);

  std::cout << "===== 初始化硬件接口结束 =====" << std::endl;
  return hardware_interface::CallbackReturn::SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 导出状态接口
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<hardware_interface::StateInterface> RRBotHardware::export_state_interfaces()
{
  std::cout << "===== 导出状态接口 =====" << std::endl;
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // std::cout << "hw_states_[状态接口][之前]: ";
  // print_vector_double(hw_states_);

  for (uint i = 0; i < hw_states_.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_states_[i]));
  }

  std::cout << "hw_states_[状态接口]: ";
  print_vector_double(hw_states_);

  std::cout << "===== 导出状态接口结束 =====" << std::endl;
  return state_interfaces;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 导出命令接口
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<hardware_interface::CommandInterface> RRBotHardware::export_command_interfaces()
{
  std::cout << "===== 导出命令接口 =====" << std::endl;
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // std::cout << "hw_commands_[命令接口][之前]: ";
  // print_vector_double(hw_commands_);

  for (uint i = 0; i < hw_commands_.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]));
  }

  std::cout << "hw_commands_[命令接口]: ";
  print_vector_double(hw_commands_);

  std::cout << "===== 导出命令接口结束 =====" << std::endl;
  return command_interfaces;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 配置硬件接口
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
hardware_interface::CallbackReturn RRBotHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::cout << "===== 配置硬件接口 =====" << std::endl;

  // 将状态接口初始化为0
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0;
  }

  // 将命令接口初始化为0
  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    hw_commands_[i] = 0;
  }

  std::cout << "hw_states_[状态接口]: ";
  print_vector_double(hw_states_);

  std::cout << "hw_commands_[命令接口]: ";
  print_vector_double(hw_commands_);

  // 等待启动时间
  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }

  std::cout << "===== 配置硬件接口结束 =====" << std::endl;
  return hardware_interface::CallbackReturn::SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 激活硬件接口
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
hardware_interface::CallbackReturn RRBotHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::cout << "===== 激活硬件接口 =====" << std::endl;

  std::cout << "===== 激活硬件接口结束 =====" << std::endl;
  return hardware_interface::CallbackReturn::SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 停用硬件接口
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
hardware_interface::CallbackReturn RRBotHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::cout << "===== 停用硬件接口 =====" << std::endl;

  std::cout << "===== 停用硬件接口结束 =====" << std::endl;
  return hardware_interface::CallbackReturn::SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 读取硬件接口
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
hardware_interface::return_type RRBotHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::cout << "===== 读取硬件接口 =====" << std::endl;

  // 读取当前时间
  const auto time = clock_->now();
  RCLCPP_INFO(get_logger(), "读取时间: %f", time.seconds());

  // 读取状态接口
  std::cout << "hw_states_[状态接口]: ";
  print_vector_double(hw_states_);

  std::cout << "===== 读取硬件接口结束 =====" << std::endl;
  return hardware_interface::return_type::OK;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 写入硬件接口
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
hardware_interface::return_type RRBotHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::cout << "===== 写入硬件接口 =====" << std::endl;

  // 写入当前时间
  const auto time = clock_->now();
  RCLCPP_INFO(get_logger(), "写入时间: %f", time.seconds());

  // 写入命令接口
  std::cout << "hw_commands_[命令接口]: ";
  print_vector_double(hw_commands_);

  std::cout << "===== 写入硬件接口结束 =====" << std::endl;
  return hardware_interface::return_type::OK;
}
}  // namespace rrbot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rrbot_hardware::RRBotHardware, hardware_interface::SystemInterface)