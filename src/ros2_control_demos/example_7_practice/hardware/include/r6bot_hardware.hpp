#ifndef R6BOT_HARDWARE_HPP_
#define R6BOT_HARDWARE_HPP_

#include "vector"
#include "string"
#include "unordered_map"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace r6bot_hardware_namespace
{

class R6BotHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;  // 初始化硬件接口
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;  // 导出状态接口
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;  // 导出命令接口
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;  // 读取硬件状态
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;  // 写入硬件命令

protected:
  hardware_interface::HardwareInfo hardware_info_;   // 硬件信息

  std::vector<double> joint_position_states_;  // 关节位置状态
  std::vector<double> joint_velocities_states_;  // 关节速度状态
  std::vector<double> ft_states_;  // 力矩传感器状态

  std::vector<double> joint_position_command_;  // 关节位置命令
  std::vector<double> joint_velocities_command_;  // 关节速度命令
  std::vector<double> ft_command_;  // 力矩传感器命令

  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {  // 关节接口
    {"position", {}}, {"velocity", {}}};
};

} // namespace r6bot_hardware_namespace

#endif  // R6BOT_HARDWARE_HPP_
