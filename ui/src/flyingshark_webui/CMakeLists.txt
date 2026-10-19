cmake_minimum_required(VERSION 3.0.2)
project(flyingshark_webui)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/webui_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY web launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
