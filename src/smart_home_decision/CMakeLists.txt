cmake_minimum_required(VERSION 3.8)
project(smart_home_decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(smart_home_interfaces REQUIRED)

include_directories(include)

add_executable(decision_node src/decision_node.cpp)
ament_target_dependencies(decision_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  smart_home_interfaces
)

install(TARGETS
  decision_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_decision test/test_decision.cpp)
  target_include_directories(test_decision PRIVATE include)
endif()

ament_package()
