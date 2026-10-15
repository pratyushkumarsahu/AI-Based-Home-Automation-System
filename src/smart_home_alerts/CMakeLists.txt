cmake_minimum_required(VERSION 3.8)
project(smart_home_alerts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(smart_home_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_executable(alert_manager_node src/alert_manager_node.cpp)
ament_target_dependencies(alert_manager_node
  rclcpp
  sensor_msgs
  visualization_msgs
  cv_bridge
  smart_home_interfaces
)
target_include_directories(alert_manager_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(alert_manager_node ${OpenCV_LIBRARIES})

install(TARGETS
  alert_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
