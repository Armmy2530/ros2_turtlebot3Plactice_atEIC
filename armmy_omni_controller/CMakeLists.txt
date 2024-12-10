cmake_minimum_required(VERSION 3.10)
project(armmy_omni_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add executable
add_executable(omni_controller src/omni_controller.cpp)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Link libraries
ament_target_dependencies(omni_controller
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  control_msgs
  tf2
  tf2_msgs
  tf2_ros
)

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

# Install targets
install(TARGETS omni_controller
  DESTINATION lib/${PROJECT_NAME})
  
# Install any launch files if applicable (optional)
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
