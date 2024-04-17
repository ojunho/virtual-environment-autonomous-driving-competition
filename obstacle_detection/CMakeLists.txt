cmake_minimum_required(VERSION 3.0.2)
project(obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  laser_geometry
  dynamic_reconfigure
)

add_message_files(
  FILES
  NearestObstacleInfo.msg
  ObstacleInfo.msg
)


generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
)


generate_dynamic_reconfigure_options(
  cfg/hyper_parameter_dynamic_static.cfg
  cfg/hyper_parameter_rotary.cfg
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES obstacle_detection
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs std_msgs geometry_msgs nav_msgs message_runtime laser_geometry dynamic_reconfigure

  DEPENDS system_lib
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(obstacle_detection_dynamic_static src/obstacle_detection_dynamic_static.cpp)
target_link_libraries(obstacle_detection_dynamic_static ${catkin_LIBRARIES})
add_dependencies(obstacle_detection_dynamic_static ${PROJECT_NAME}_gencfg)

add_executable(obstacle_detection_rotary src/obstacle_detection_rotary.cpp)
target_link_libraries(obstacle_detection_rotary ${catkin_LIBRARIES})
add_dependencies(obstacle_detection_rotary ${PROJECT_NAME}_gencfg)

