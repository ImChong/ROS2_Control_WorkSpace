#include "r6bot_controller.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// helper 函数
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void print_vector_string(const std::vector<std::string> & vec) {
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
namespace r6bot_controller_namespace
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 构造函数
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
R6BotController::R6BotController() : controller_interface::ControllerInterface() {}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 初始化
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
controller_interface::CallbackReturn R6BotController::on_init() {
  std::cout << "===== 控制器初始化开始 =====" << std::endl;

  // 自动声明
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

  std::cout << "joint_names_: ";
  print_vector_string(joint_names_);
  std::cout << "command_interface_types_: ";
  print_vector_string(command_interface_types_);
  std::cout << "state_interface_types_: ";
  print_vector_string(state_interface_types_);

  std::cout << "===== 控制器初始化完成 =====" << std::endl;
  return controller_interface::CallbackReturn::SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 配置
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
controller_interface::CallbackReturn R6BotController::on_configure(const rclcpp_lifecycle::State & previous_state) {
  std::cout << "===== 控制器配置开始 =====" << std::endl;
  (void)previous_state;

  std::cout << "===== 控制器配置完成 =====" << std::endl;
  return controller_interface::CallbackReturn::SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 命令接口配置
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
controller_interface::InterfaceConfiguration R6BotController::command_interface_configuration() const {
  static int command_interface_config_call_count = 0;
  const auto timestamp = this->get_node()->now().seconds();

  std::cout << "===== [Call #" << ++command_interface_config_call_count << " at " << timestamp << "s] 控制器命令接口配置开始 =====" << std::endl;

  // 配置命令接口
  controller_interface::InterfaceConfiguration conf = {controller_interface::interface_configuration_type::INDIVIDUAL, {}};

  // 预留空间
  conf.names.reserve(joint_names_.size() * command_interface_types_.size());

  // 遍历关节
  for (const auto & joint_name : joint_names_) {
    // 遍历命令接口类型
    for (const auto & interface_type : command_interface_types_) {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  std::string names_str = "";
  for (const auto & name : conf.names) {
    if (!names_str.empty()) names_str += ", ";
    names_str += name;
  }
  std::cout << "command interface configuration: " << names_str << std::endl;
  std::cout << "===== 控制器命令接口配置完成 =====" << std::endl;
  return conf;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 状态接口配置
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
controller_interface::InterfaceConfiguration R6BotController::state_interface_configuration() const {
  static int state_interface_config_call_count = 0;
  const auto timestamp = this->get_node()->now().seconds();

  std::cout << "===== [Call #" << ++state_interface_config_call_count << " at " << timestamp << "s] 控制器状态接口配置开始 =====" << std::endl;

  // 配置状态接口
  controller_interface::InterfaceConfiguration conf = {controller_interface::interface_configuration_type::INDIVIDUAL, {}};

  // 预留空间
  conf.names.reserve(joint_names_.size() * state_interface_types_.size());

  // 遍历关节
  for (const auto & joint_name : joint_names_) {
    // 遍历状态接口类型
    for (const auto & interface_type : state_interface_types_) {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  // 打印状态接口名称
  std::string names_str = "";
  for (const auto & name : conf.names) {
    if (!names_str.empty()) names_str += ", ";
    names_str += name;
  }
  std::cout << "state interface configuration: " << names_str << std::endl;
  std::cout << "===== 控制器状态接口配置完成 =====" << std::endl;
  return conf;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 激活
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
controller_interface::CallbackReturn R6BotController::on_activate(const rclcpp_lifecycle::State & previous_state) {
  std::cout << "===== 控制器激活开始 =====" << std::endl;
  (void)previous_state;

  // 清空接口
  joint_position_command_interface_.clear();
  joint_velocity_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();

  // 分配命令接口
  for (auto & interface : command_interfaces_)
  {
    command_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  // 分配状态接口
  for (auto & interface : state_interfaces_)
  {
    state_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  std::cout << "===== 控制器激活完成 =====" << std::endl;
  return controller_interface::CallbackReturn::SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 更新
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
controller_interface::return_type R6BotController::update(const rclcpp::Time & time, const rclcpp::Duration & period) {
  std::cout << "===== 控制器更新开始 =====" << std::endl;
  std::cout << "time: " << std::fixed << std::setprecision(3) << time.seconds() << std::endl;
  std::cout << "period: " << std::fixed << std::setprecision(3) << period.seconds() << std::endl;

  std::cout << "===== 控制器更新完成 =====" << std::endl;
  return controller_interface::return_type::OK;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 禁用
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
controller_interface::CallbackReturn R6BotController::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
  std::cout << "===== 控制器禁用开始 =====" << std::endl;
  (void)previous_state;
  release_interfaces(); // 释放接口

  std::cout << "===== 控制器禁用完成 =====" << std::endl;
  return controller_interface::CallbackReturn::SUCCESS;
}

}   // namespace r6bot_controller_namespace

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(r6bot_controller_namespace::R6BotController, controller_interface::ControllerInterface)
