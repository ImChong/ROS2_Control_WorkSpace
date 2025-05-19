#ifndef R6BOT_HARDWARE_HPP_
#define R6BOT_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace r6bot_hardware_namespace
{

class R6BotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(R6BotHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  rclcpp::Logger get_logger() const { return *logger_; }
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:
  hardware_interface::HardwareInfo hardware_info_;   // 硬件信息

  std::shared_ptr<rclcpp::Logger> logger_;            // 日志
  rclcpp::Clock::SharedPtr clock_;                    // 时钟

  std::vector<double> hw_states_;            // 状态
  std::vector<double> hw_commands_;          // 命令
};

} // namespace r6bot_hardware_namespace

#endif  // R6BOT_HARDWARE_HPP_
