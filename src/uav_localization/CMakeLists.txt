cmake_minimum_required(VERSION 3.8)
project(uav_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common io registration)

add_executable(uav_localization_node src/uav_localization_node.cpp)
target_include_directories(uav_localization_node PUBLIC
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(uav_localization_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)
target_link_libraries(uav_localization_node ${PCL_LIBRARIES})

install(TARGETS
  uav_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
