# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(FATAL_ERROR "Could not find messages which '/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/srv/SetControlMode.srv' depends on. Did you forget to specify generate_messages(DEPENDENCIES ...)?
Cannot locate message [ControlMode]: unknown package [bluerov2_control] on search path [{'bluerov2_control_real': ['/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg', '/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg'], 'geographic_msgs': ['/opt/ros/noetic/share/geographic_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/noetic/share/geometry_msgs/cmake/../msg'], 'uuv_control_msgs': ['/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg'], 'actionlib_msgs': ['/opt/ros/noetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/noetic/share/std_msgs/cmake/../msg'], 'uuid_msgs': ['/opt/ros/noetic/share/uuid_msgs/cmake/../msg']}]")
message(STATUS "bluerov2_control_real: 9 messages, 2 services")

set(MSG_I_FLAGS "-Ibluerov2_control_real:/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg;-Ibluerov2_control_real:/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg;-Igeographic_msgs:/opt/ros/noetic/share/geographic_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Iuuv_control_msgs:/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Iuuid_msgs:/opt/ros/noetic/share/uuid_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(bluerov2_control_real_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/ControlMode.msg" NAME_WE)
add_custom_target(_bluerov2_control_real_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control_real" "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/ControlMode.msg" ""
)

get_filename_component(_filename "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/Autopilot.msg" NAME_WE)
add_custom_target(_bluerov2_control_real_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control_real" "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/Autopilot.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsAction.msg" NAME_WE)
add_custom_target(_bluerov2_control_real_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control_real" "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsAction.msg" "actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:bluerov2_control_real/FollowWaypointsResult:bluerov2_control_real/FollowWaypointsActionGoal:bluerov2_control_real/FollowWaypointsActionResult:bluerov2_control_real/FollowWaypointsGoal:bluerov2_control_real/FollowWaypointsFeedback:std_msgs/Time:bluerov2_control_real/FollowWaypointsActionFeedback:std_msgs/Header:uuv_control_msgs/Waypoint:geometry_msgs/Point:uuv_control_msgs/WaypointSet"
)

get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionGoal.msg" NAME_WE)
add_custom_target(_bluerov2_control_real_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control_real" "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionGoal.msg" "actionlib_msgs/GoalID:bluerov2_control_real/FollowWaypointsGoal:std_msgs/Time:std_msgs/Header:uuv_control_msgs/Waypoint:geometry_msgs/Point:uuv_control_msgs/WaypointSet"
)

get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionResult.msg" NAME_WE)
add_custom_target(_bluerov2_control_real_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control_real" "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionResult.msg" "actionlib_msgs/GoalStatus:std_msgs/Header:actionlib_msgs/GoalID:bluerov2_control_real/FollowWaypointsResult"
)

get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionFeedback.msg" NAME_WE)
add_custom_target(_bluerov2_control_real_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control_real" "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionFeedback.msg" "actionlib_msgs/GoalStatus:std_msgs/Header:actionlib_msgs/GoalID:bluerov2_control_real/FollowWaypointsFeedback"
)

get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg" NAME_WE)
add_custom_target(_bluerov2_control_real_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control_real" "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg" "std_msgs/Time:std_msgs/Header:uuv_control_msgs/Waypoint:uuv_control_msgs/WaypointSet:geometry_msgs/Point"
)

get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg" NAME_WE)
add_custom_target(_bluerov2_control_real_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control_real" "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg" ""
)

get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg" NAME_WE)
add_custom_target(_bluerov2_control_real_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control_real" "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg" ""
)

get_filename_component(_filename "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/srv/ConvertGeoPoints.srv" NAME_WE)
add_custom_target(_bluerov2_control_real_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control_real" "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/srv/ConvertGeoPoints.srv" "geographic_msgs/GeoPoint:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/ControlMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_cpp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/Autopilot.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_cpp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionGoal.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionResult.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_cpp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_cpp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_cpp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_cpp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_cpp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_cpp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control_real
)

### Generating Services
_generate_srv_cpp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/srv/ConvertGeoPoints.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geographic_msgs/cmake/../msg/GeoPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control_real
)

### Generating Module File
_generate_module_cpp(bluerov2_control_real
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control_real
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(bluerov2_control_real_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(bluerov2_control_real_generate_messages bluerov2_control_real_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/ControlMode.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_cpp _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/Autopilot.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_cpp _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsAction.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_cpp _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_cpp _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionResult.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_cpp _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_cpp _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_cpp _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_cpp _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_cpp _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/srv/ConvertGeoPoints.srv" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_cpp _bluerov2_control_real_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bluerov2_control_real_gencpp)
add_dependencies(bluerov2_control_real_gencpp bluerov2_control_real_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bluerov2_control_real_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/ControlMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_eus(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/Autopilot.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_eus(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionGoal.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionResult.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_eus(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_eus(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_eus(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_eus(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_eus(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_eus(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control_real
)

### Generating Services
_generate_srv_eus(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/srv/ConvertGeoPoints.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geographic_msgs/cmake/../msg/GeoPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control_real
)

### Generating Module File
_generate_module_eus(bluerov2_control_real
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control_real
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(bluerov2_control_real_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(bluerov2_control_real_generate_messages bluerov2_control_real_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/ControlMode.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_eus _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/Autopilot.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_eus _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsAction.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_eus _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_eus _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionResult.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_eus _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_eus _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_eus _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_eus _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_eus _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/srv/ConvertGeoPoints.srv" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_eus _bluerov2_control_real_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bluerov2_control_real_geneus)
add_dependencies(bluerov2_control_real_geneus bluerov2_control_real_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bluerov2_control_real_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/ControlMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_lisp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/Autopilot.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_lisp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionGoal.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionResult.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_lisp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_lisp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_lisp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_lisp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_lisp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_lisp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control_real
)

### Generating Services
_generate_srv_lisp(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/srv/ConvertGeoPoints.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geographic_msgs/cmake/../msg/GeoPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control_real
)

### Generating Module File
_generate_module_lisp(bluerov2_control_real
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control_real
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(bluerov2_control_real_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(bluerov2_control_real_generate_messages bluerov2_control_real_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/ControlMode.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_lisp _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/Autopilot.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_lisp _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsAction.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_lisp _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_lisp _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionResult.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_lisp _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_lisp _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_lisp _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_lisp _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_lisp _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/srv/ConvertGeoPoints.srv" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_lisp _bluerov2_control_real_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bluerov2_control_real_genlisp)
add_dependencies(bluerov2_control_real_genlisp bluerov2_control_real_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bluerov2_control_real_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/ControlMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_nodejs(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/Autopilot.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_nodejs(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionGoal.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionResult.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_nodejs(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_nodejs(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_nodejs(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_nodejs(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_nodejs(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_nodejs(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control_real
)

### Generating Services
_generate_srv_nodejs(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/srv/ConvertGeoPoints.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geographic_msgs/cmake/../msg/GeoPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control_real
)

### Generating Module File
_generate_module_nodejs(bluerov2_control_real
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control_real
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(bluerov2_control_real_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(bluerov2_control_real_generate_messages bluerov2_control_real_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/ControlMode.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_nodejs _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/Autopilot.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_nodejs _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsAction.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_nodejs _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_nodejs _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionResult.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_nodejs _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_nodejs _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_nodejs _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_nodejs _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_nodejs _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/srv/ConvertGeoPoints.srv" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_nodejs _bluerov2_control_real_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bluerov2_control_real_gennodejs)
add_dependencies(bluerov2_control_real_gennodejs bluerov2_control_real_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bluerov2_control_real_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/ControlMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_py(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/Autopilot.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_py(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionGoal.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionResult.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_py(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_py(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_py(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_py(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_py(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control_real
)
_generate_msg_py(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control_real
)

### Generating Services
_generate_srv_py(bluerov2_control_real
  "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/srv/ConvertGeoPoints.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geographic_msgs/cmake/../msg/GeoPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control_real
)

### Generating Module File
_generate_module_py(bluerov2_control_real
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control_real
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(bluerov2_control_real_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(bluerov2_control_real_generate_messages bluerov2_control_real_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/ControlMode.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_py _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/msg/Autopilot.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_py _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsAction.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_py _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_py _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionResult.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_py _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsActionFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_py _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_py _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsResult.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_py _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/devel/share/bluerov2_control_real/msg/FollowWaypointsFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_py _bluerov2_control_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/test-8/eight-thurster/src/real_test/bluerov2_real/bluerov2_control_real/srv/ConvertGeoPoints.srv" NAME_WE)
add_dependencies(bluerov2_control_real_generate_messages_py _bluerov2_control_real_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bluerov2_control_real_genpy)
add_dependencies(bluerov2_control_real_genpy bluerov2_control_real_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bluerov2_control_real_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control_real)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control_real
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geographic_msgs_generate_messages_cpp)
  add_dependencies(bluerov2_control_real_generate_messages_cpp geographic_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(bluerov2_control_real_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET uuv_control_msgs_generate_messages_cpp)
  add_dependencies(bluerov2_control_real_generate_messages_cpp uuv_control_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(bluerov2_control_real_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control_real)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control_real
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geographic_msgs_generate_messages_eus)
  add_dependencies(bluerov2_control_real_generate_messages_eus geographic_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(bluerov2_control_real_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET uuv_control_msgs_generate_messages_eus)
  add_dependencies(bluerov2_control_real_generate_messages_eus uuv_control_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(bluerov2_control_real_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control_real)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control_real
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geographic_msgs_generate_messages_lisp)
  add_dependencies(bluerov2_control_real_generate_messages_lisp geographic_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(bluerov2_control_real_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET uuv_control_msgs_generate_messages_lisp)
  add_dependencies(bluerov2_control_real_generate_messages_lisp uuv_control_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(bluerov2_control_real_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control_real)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control_real
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geographic_msgs_generate_messages_nodejs)
  add_dependencies(bluerov2_control_real_generate_messages_nodejs geographic_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(bluerov2_control_real_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET uuv_control_msgs_generate_messages_nodejs)
  add_dependencies(bluerov2_control_real_generate_messages_nodejs uuv_control_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(bluerov2_control_real_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control_real)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control_real\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control_real
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geographic_msgs_generate_messages_py)
  add_dependencies(bluerov2_control_real_generate_messages_py geographic_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(bluerov2_control_real_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET uuv_control_msgs_generate_messages_py)
  add_dependencies(bluerov2_control_real_generate_messages_py uuv_control_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(bluerov2_control_real_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
