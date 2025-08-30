cmake_minimum_required(VERSION 3.0.2)
project(real_test)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    mavros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/pubreal.py
  scripts/publish2real.py
  scripts/publish2real_copy.py
  scripts/stop.py
  scripts/caculate_forece_PWM.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 