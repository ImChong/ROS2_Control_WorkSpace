// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros2_control_demo_example_1/rrbot.hpp"

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
#include <iostream>

namespace ros2_control_demo_example_1
{

// on_init 主要功能：初始化硬件接口hw_states_和hw_commands_，设置日志和时钟
hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.RRBot"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // 打印所有info
  std::cout << "info_.name: " << info_.name << std::endl;
  std::cout << "info_.type: " << info_.type << std::endl;
  std::cout << "info_.hardware_class_type: " << info_.hardware_class_type << std::endl;
  std::cout << "info_.joints.size(): " << info_.joints.size() << std::endl;
  for (const auto & joint : info_.joints)
  {
    std::cout << "====================" << std::endl;
    std::cout << "\tjoint.name: " << joint.name << std::endl;
    std::cout << "\tjoint.type: " << joint.type << std::endl;
    std::cout << "\tjoint.command_interfaces.size(): " << joint.command_interfaces.size() << std::endl;
    for (const auto & command_interface : joint.command_interfaces)
    {
      std::cout << "\t\tcommand_interface.name: " << command_interface.name << std::endl;
      std::cout << "\t\tcommand_interface.min: " << command_interface.min << std::endl;
      std::cout << "\t\tcommand_interface.max: " << command_interface.max << std::endl;
      std::cout << "\t\tcommand_interface.initial_value: " << command_interface.initial_value << std::endl;
      std::cout << "\t\tcommand_interface.data_type: " << command_interface.data_type << std::endl;
      std::cout << "\t\tcommand_interface.size: " << command_interface.size << std::endl;
    }
    std::cout << "\tjoint.state_interfaces.size(): " << joint.state_interfaces.size() << std::endl;
    for (const auto & state_interface : joint.state_interfaces)
    {
      std::cout << "\t\tstate_interface.name: " << state_interface.name << std::endl;
      std::cout << "\t\tstate_interface.min: " << state_interface.min << std::endl;
      std::cout << "\t\tstate_interface.max: " << state_interface.max << std::endl;
      std::cout << "\t\tstate_interface.initial_value: " << state_interface.initial_value << std::endl;
      std::cout << "\t\tstate_interface.data_type: " << state_interface.data_type << std::endl;
      std::cout << "\t\tstate_interface.size: " << state_interface.size << std::endl;
    }
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);

  // 打印所有参数
  std::cout << "hw_start_sec_: " << hw_start_sec_ << std::endl;
  std::cout << "hw_stop_sec_: " << hw_stop_sec_ << std::endl;
  std::cout << "hw_slowdown_: " << hw_slowdown_ << std::endl;

  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// on_configure 主要功能：将状态和命令接口初始化为默认值，在此默认值为0
hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  std::cout << "hw_start_sec_: " << hw_start_sec_ << std::endl;
  std::cout << "hw_states_: ";
  for (auto val : hw_states_) {
    std::cout << val << " ";
  }
  std::cout << std::endl;
  std::cout << "hw_commands_: ";
  for (auto val : hw_commands_) {
    std::cout << val << " ";
  }
  std::cout << std::endl;

  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
  }

  std::cout << "hw_states_: ";
  for (auto val : hw_states_) {
    std::cout << val << " ";
  }
  std::cout << std::endl;
  std::cout << "hw_commands_: ";
  for (auto val : hw_commands_) {
    std::cout << val << " ";
  }
  std::cout << std::endl;

  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RRBotSystemPositionOnlyHardware::export_state_interfaces()
{
  std::cout << "=====" << std::endl;
  std::cout << "RRBotSystemPositionOnlyHardware::export_state_interfaces()" << std::endl;

  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    std::cout << "\tinfo_.joints[" << i << "].name: " << info_.joints[i].name << std::endl;
    std::cout << "\thardware_interface::HW_IF_POSITION: " << hardware_interface::HW_IF_POSITION << std::endl;
    std::cout << "\thw_states_[" << i << "]: " << hw_states_[i] << std::endl;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  std::cout << "=====" << std::endl;
  std::cout << "finished export_state_interfaces()" << std::endl;
  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>
RRBotSystemPositionOnlyHardware::export_command_interfaces()
{
  std::cout << "=====" << std::endl;
  std::cout << "RRBotSystemPositionOnlyHardware::export_command_interfaces()" << std::endl;

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    std::cout << "\tinfo_.joints[" << i << "].name: " << info_.joints[i].name << std::endl;
    std::cout << "\thardware_interface::HW_IF_POSITION: " << hardware_interface::HW_IF_POSITION << std::endl;
    std::cout << "\thw_commands_[" << i << "]: " << hw_commands_[i] << std::endl;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  std::cout << "=====" << std::endl;
  std::cout << "finished export_command_interfaces()" << std::endl;
  return command_interfaces;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Reading states:";

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    // Simulate RRBot's movement
    hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;

    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << hw_states_[i] << " for joint '" << info_.joints[i].name << "'";
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Writing commands:";

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << hw_commands_[i] << " for joint '" << info_.joints[i].name << "'";
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_1

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_1::RRBotSystemPositionOnlyHardware, hardware_interface::SystemInterface)
