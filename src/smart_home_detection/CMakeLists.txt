cmake_minimum_required(VERSION 3.8)
project(smart_home_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(smart_home_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs dnn)

add_executable(detection_node src/detection_node.cpp)
ament_target_dependencies(detection_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  smart_home_interfaces
  ament_index_cpp
)
target_include_directories(detection_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(detection_node ${OpenCV_LIBRARIES})

install(TARGETS
  detection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/download_models.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
