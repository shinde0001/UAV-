cmake_minimum_required(VERSION 3.8)
project(uav_obstacle_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common io filters)

add_executable(local_obstacle_filter src/local_obstacle_filter.cpp)
target_include_directories(local_obstacle_filter PUBLIC ${PCL_INCLUDE_DIRS})
ament_target_dependencies(local_obstacle_filter rclcpp sensor_msgs pcl_conversions)
target_link_libraries(local_obstacle_filter ${PCL_LIBRARIES})

install(TARGETS local_obstacle_filter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
