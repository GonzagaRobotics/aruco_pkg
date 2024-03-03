cmake_minimum_required(VERSION 3.5)
project(aruco_pkg)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(CodeCoverage.cmake)
APPEND_COVERAGE_COMPILER_FLAGS()

set(COVERAGE_LCOV_EXCLUDES 'launch/*' 'inputs/*' 'src/injector.cpp')
SETUP_TARGET_FOR_COVERAGE_LCOV(NAME coverage_report EXECUTABLE ctest --output-on-failure)

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
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(aruco_sensor src/arucoNode.cpp)
ament_target_dependencies(aruco_sensor
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
)
target_link_libraries(aruco_sensor ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
target_include_directories(aruco_sensor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  aruco_sensor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch inputs
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
