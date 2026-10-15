cmake_minimum_required(VERSION 3.8)
project(smart_home_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(smart_home_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Gas Sensor Node (with visual plume emitter, cloud fallback, and RViz markers)
add_executable(gas_sensor_node src/gas_sensor_node.cpp)
ament_target_dependencies(gas_sensor_node
  rclcpp
  nav_msgs
  std_msgs
  smart_home_interfaces
  visualization_msgs
)
target_link_libraries(gas_sensor_node
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

# Person Motion Node (animates walking routes for family members and outsider)
add_executable(person_motion_node src/person_motion_node.cpp)
ament_target_dependencies(person_motion_node
  rclcpp
  std_msgs
)
target_link_libraries(person_motion_node
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS
  gas_sensor_node
  person_motion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY worlds models config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
