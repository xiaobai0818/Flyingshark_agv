cmake_minimum_required(VERSION 3.0.2)
project(openctr_driver)

add_compile_options(-Wall -Wextra -std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  serial
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(openctr_driver_node src/openctr_driver.cpp)
target_link_libraries(openctr_driver_node ${catkin_LIBRARIES})

install(TARGETS openctr_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
