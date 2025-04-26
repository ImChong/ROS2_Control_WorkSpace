cmake_minimum_required(VERSION 3.20)
project(ros2_control_demos_example_1_display_urdf LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rviz2
  xacro
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# COMPILE
target_compile_features(ros2_control_demos_example_1_display_urdf PUBLIC cxx_std_17)

ament_target_dependencies(
  ros2_control_demos_example_1_display_urdf PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS})

# INSTALL
install(
  DIRECTORY description/launch description/rviz description/urdf
  DESTINATION share/ros2_control_demos_example_1_display_urdf
)

install(TARGETS ros2_control_demos_example_1_display_urdf
  EXPORT export_ros2_control_demos_example_1_display_urdf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# EXPORTS
ament_export_targets(export_ros2_control_demos_example_1_display_urdf HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# PACKAGE
ament_package()