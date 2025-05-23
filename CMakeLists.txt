cmake_minimum_required(VERSION 3.5)
project(xv_11_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rmw_cyclonedds_cpp REQUIRED)


add_executable(xv_11_driver src/xv_11_driver.cpp src/xv11_laser.cpp)
ament_target_dependencies(xv_11_driver rclcpp std_msgs sensor_msgs rmw_cyclonedds_cpp)

#target_include_directories(xv_11_driver
#  PUBLIC
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#    $<INSTALL_INTERFACE:include>)

install(TARGETS
  xv_11_driver
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
