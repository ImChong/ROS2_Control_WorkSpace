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
  // 初始化硬件接口
  hardware_interface::CallbackReturn RRBotHardware::on_init(const hardware_interface::HardwareInfo & info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // 配置硬件接口
  hardware_interface::CallbackReturn RRBotHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // 导出状态接口
  std::vector<hardware_interface::StateInterface> RRBotHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    return state_interfaces;
  }

  // 导出命令接口
  std::vector<hardware_interface::CommandInterface> RRBotHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    return command_interfaces;
  }

  // 激活硬件接口
  hardware_interface::CallbackReturn RRBotHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // 停用硬件接口
  hardware_interface::CallbackReturn RRBotHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // 读取硬件接口
  hardware_interface::return_type RRBotHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    return hardware_interface::return_type::OK;
  }

  // 写入硬件接口
  hardware_interface::return_type RRBotHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    return hardware_interface::return_type::OK;
  }
}  // namespace rrbot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rrbot_hardware::RRBotHardware, hardware_interface::SystemInterface)