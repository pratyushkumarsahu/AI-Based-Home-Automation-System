cmake_minimum_required(VERSION 3.8)
project(smart_home_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_pipeline_launch.py)
endif()

ament_package()
