# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "bluerov2_dobmpc: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ibluerov2_dobmpc:/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(bluerov2_dobmpc_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Reference.msg" NAME_WE)
add_custom_target(_bluerov2_dobmpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_dobmpc" "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Reference.msg" "geometry_msgs/Point:geometry_msgs/Pose:geometry_msgs/Vector3:geometry_msgs/Twist:std_msgs/Header:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Pose.msg" NAME_WE)
add_custom_target(_bluerov2_dobmpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_dobmpc" "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Pose.msg" "geometry_msgs/Point:geometry_msgs/Pose:geometry_msgs/Vector3:geometry_msgs/Twist:std_msgs/Header:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(bluerov2_dobmpc
  "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Reference.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_dobmpc
)
_generate_msg_cpp(bluerov2_dobmpc
  "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_dobmpc
)

### Generating Services

### Generating Module File
_generate_module_cpp(bluerov2_dobmpc
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_dobmpc
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(bluerov2_dobmpc_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(bluerov2_dobmpc_generate_messages bluerov2_dobmpc_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Reference.msg" NAME_WE)
add_dependencies(bluerov2_dobmpc_generate_messages_cpp _bluerov2_dobmpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Pose.msg" NAME_WE)
add_dependencies(bluerov2_dobmpc_generate_messages_cpp _bluerov2_dobmpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bluerov2_dobmpc_gencpp)
add_dependencies(bluerov2_dobmpc_gencpp bluerov2_dobmpc_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bluerov2_dobmpc_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(bluerov2_dobmpc
  "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Reference.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_dobmpc
)
_generate_msg_eus(bluerov2_dobmpc
  "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_dobmpc
)

### Generating Services

### Generating Module File
_generate_module_eus(bluerov2_dobmpc
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_dobmpc
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(bluerov2_dobmpc_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(bluerov2_dobmpc_generate_messages bluerov2_dobmpc_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Reference.msg" NAME_WE)
add_dependencies(bluerov2_dobmpc_generate_messages_eus _bluerov2_dobmpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Pose.msg" NAME_WE)
add_dependencies(bluerov2_dobmpc_generate_messages_eus _bluerov2_dobmpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bluerov2_dobmpc_geneus)
add_dependencies(bluerov2_dobmpc_geneus bluerov2_dobmpc_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bluerov2_dobmpc_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(bluerov2_dobmpc
  "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Reference.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_dobmpc
)
_generate_msg_lisp(bluerov2_dobmpc
  "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_dobmpc
)

### Generating Services

### Generating Module File
_generate_module_lisp(bluerov2_dobmpc
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_dobmpc
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(bluerov2_dobmpc_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(bluerov2_dobmpc_generate_messages bluerov2_dobmpc_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Reference.msg" NAME_WE)
add_dependencies(bluerov2_dobmpc_generate_messages_lisp _bluerov2_dobmpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Pose.msg" NAME_WE)
add_dependencies(bluerov2_dobmpc_generate_messages_lisp _bluerov2_dobmpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bluerov2_dobmpc_genlisp)
add_dependencies(bluerov2_dobmpc_genlisp bluerov2_dobmpc_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bluerov2_dobmpc_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(bluerov2_dobmpc
  "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Reference.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_dobmpc
)
_generate_msg_nodejs(bluerov2_dobmpc
  "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_dobmpc
)

### Generating Services

### Generating Module File
_generate_module_nodejs(bluerov2_dobmpc
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_dobmpc
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(bluerov2_dobmpc_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(bluerov2_dobmpc_generate_messages bluerov2_dobmpc_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Reference.msg" NAME_WE)
add_dependencies(bluerov2_dobmpc_generate_messages_nodejs _bluerov2_dobmpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Pose.msg" NAME_WE)
add_dependencies(bluerov2_dobmpc_generate_messages_nodejs _bluerov2_dobmpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bluerov2_dobmpc_gennodejs)
add_dependencies(bluerov2_dobmpc_gennodejs bluerov2_dobmpc_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bluerov2_dobmpc_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(bluerov2_dobmpc
  "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Reference.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_dobmpc
)
_generate_msg_py(bluerov2_dobmpc
  "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_dobmpc
)

### Generating Services

### Generating Module File
_generate_module_py(bluerov2_dobmpc
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_dobmpc
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(bluerov2_dobmpc_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(bluerov2_dobmpc_generate_messages bluerov2_dobmpc_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Reference.msg" NAME_WE)
add_dependencies(bluerov2_dobmpc_generate_messages_py _bluerov2_dobmpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zeb/eight-thurster/src/bluerov2/bluerov2_dobmpc/backup/msg/Pose.msg" NAME_WE)
add_dependencies(bluerov2_dobmpc_generate_messages_py _bluerov2_dobmpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bluerov2_dobmpc_genpy)
add_dependencies(bluerov2_dobmpc_genpy bluerov2_dobmpc_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bluerov2_dobmpc_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_dobmpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_dobmpc
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(bluerov2_dobmpc_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(bluerov2_dobmpc_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(bluerov2_dobmpc_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_dobmpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_dobmpc
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(bluerov2_dobmpc_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(bluerov2_dobmpc_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(bluerov2_dobmpc_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_dobmpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_dobmpc
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(bluerov2_dobmpc_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(bluerov2_dobmpc_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(bluerov2_dobmpc_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_dobmpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_dobmpc
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(bluerov2_dobmpc_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(bluerov2_dobmpc_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(bluerov2_dobmpc_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_dobmpc)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_dobmpc\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_dobmpc
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(bluerov2_dobmpc_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(bluerov2_dobmpc_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(bluerov2_dobmpc_generate_messages_py geometry_msgs_generate_messages_py)
endif()
