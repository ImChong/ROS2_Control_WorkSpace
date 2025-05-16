cmake_minimum_required(VERSION 3.20)
project(ros2_control_demo_example_1_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# DEPENDENCIES #########################################################################################################
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  pluginlib
  rclcpp
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE #############################################################################################################
add_library(
  ros2_control_demo_example_1_hardware
  SHARED
  hardware/rrbot.cpp
)
target_compile_features(ros2_control_demo_example_1_hardware PUBLIC cxx_std_17)
target_include_directories(ros2_control_demo_example_1_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/hardware>
)
ament_target_dependencies(
  ros2_control_demo_example_1_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins ##############################################################################################
pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_example_1_hardware.xml)

# INSTALL ##############################################################################################################
install(
  DIRECTORY description/
  DESTINATION share/ros2_control_demo_example_1_hardware/description
)
install(
  DIRECTORY launch/
  DESTINATION share/ros2_control_demo_example_1_hardware/launch
)
install(
  DIRECTORY controller/
  DESTINATION share/ros2_control_demo_example_1_hardware/controller
)
install(
  DIRECTORY hardware/include/
  DESTINATION include/hardware/
)
install(TARGETS ros2_control_demo_example_1_hardware
  EXPORT export_ros2_control_demo_example_1_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# TEST #################################################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# EXPORTS ##############################################################################################################
ament_export_targets(export_ros2_control_demo_example_1_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
