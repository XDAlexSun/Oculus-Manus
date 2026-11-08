cmake_minimum_required(VERSION 3.8)
project(robot_teleop)

# retargeting.hpp uses `inline const` variables, which need C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(teleop_node
  src/teleop_node.cpp
)
target_include_directories(teleop_node PRIVATE src)

ament_target_dependencies(
  teleop_node
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
)

install(
  TARGETS teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # plain-assert math test, registered as a CTest so `colcon test` runs it
  add_executable(test_retargeting test/test_retargeting.cpp)
  target_include_directories(test_retargeting PRIVATE src)
  add_test(NAME test_retargeting COMMAND test_retargeting)

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
