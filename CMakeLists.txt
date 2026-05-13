cmake_minimum_required(VERSION 3.0.2)
project(safe_valley_exp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  mavros_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs mavros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/safe_flock_real.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/safe_flock.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
