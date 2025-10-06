cmake_minimum_required(VERSION 3.5)
project(aro_motor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

install(
    PROGRAMS
    scripts/motor_driver.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
