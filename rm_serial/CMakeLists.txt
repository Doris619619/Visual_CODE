cmake_minimum_required(VERSION 3.8)
project(rm_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_definitions(-Wall -Werror)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
ament_auto_find_build_dependencies()

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

#ament_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/serial_driver.cpp
  src/rm_serial.cpp
  src/gimbal_manager.cpp
  src/crc.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_serial_driver::EXTSerialDriver
  EXECUTABLE ${PROJECT_NAME}_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_serial_driver::GimbalManagerNode
  EXECUTABLE gimbal_manager_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
