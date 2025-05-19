cmake_minimum_required(VERSION 3.20)
project(r6bot_control_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# DEPENDENCIES #########################################################################################################
set(HARDWARE_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)
set(TRAJ_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${HARDWARE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${TRAJ_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE #############################################################################################################
add_executable(generate_trajectory trajectory_generator/main.cpp)

ament_target_dependencies(
  generate_trajectory PUBLIC
  ${TRAJ_GEN_INCLUDE_DEPENDS}
)

add_library(
  ${PROJECT_NAME}
  SHARED
  hardware/r6bot_hardware.cpp
  controller/r6bot_controller.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  ${HARDWARE_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
)

# Export PLUGINS #######################################################################################################

pluginlib_export_plugin_description_file(hardware_interface r6bot_control_system_plugins.xml)     # Export hardware plugins
pluginlib_export_plugin_description_file(controller_interface r6bot_control_system_plugins.xml)   # Export controller plugins

# INSTALL ##############################################################################################################
install(
  DIRECTORY hardware/include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY controller/include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS generate_trajectory
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
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
ament_package()
