#ifndef R6BOT_CONTROLLER_HPP_
#define R6BOT_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"

namespace r6bot_controller_namespace
{

class R6BotController : public controller_interface::ControllerInterface
{
public:
  R6BotController();
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;
};
}   // namespace r6bot_controller_namespace


#endif  // R6BOT_CONTROLLER_HPP_