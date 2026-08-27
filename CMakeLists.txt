cmake_minimum_required(VERSION 3.0.2)
project(safe_valley_exp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  mavros_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs mavros_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/safe_flock_main.py
  scripts/wait_mavros.py
  scripts/submode_publisher.py
  scripts/offboard_takeoff_15.py
  scripts/gps_bias_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_gps_bias.py)
  catkin_add_nosetests(test/test_gps_bias_wiring.py)
  add_rostest(test/gps_bias_no_reference.test)
endif()
