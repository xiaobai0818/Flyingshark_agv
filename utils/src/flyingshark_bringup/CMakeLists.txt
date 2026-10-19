cmake_minimum_required(VERSION 3.0.2)
project(flyingshark_bringup)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/scan_filter.py scripts/rgb_cloud_downsample.py scripts/depth_cloud_downsample.py scripts/height_map_overlay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
