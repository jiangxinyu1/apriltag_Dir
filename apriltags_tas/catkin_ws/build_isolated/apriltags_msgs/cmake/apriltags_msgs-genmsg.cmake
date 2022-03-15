# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "apriltags_msgs: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iapriltags_msgs:/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(apriltags_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTag.msg" NAME_WE)
add_custom_target(_apriltags_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "apriltags_msgs" "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTag.msg" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTagDetections.msg" NAME_WE)
add_custom_target(_apriltags_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "apriltags_msgs" "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTagDetections.msg" "apriltags_msgs/AprilTag:geometry_msgs/Pose:geometry_msgs/Point:geometry_msgs/Quaternion:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(apriltags_msgs
  "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTag.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/apriltags_msgs
)
_generate_msg_cpp(apriltags_msgs
  "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTagDetections.msg"
  "${MSG_I_FLAGS}"
  "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTag.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/apriltags_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(apriltags_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/apriltags_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(apriltags_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(apriltags_msgs_generate_messages apriltags_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTag.msg" NAME_WE)
add_dependencies(apriltags_msgs_generate_messages_cpp _apriltags_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTagDetections.msg" NAME_WE)
add_dependencies(apriltags_msgs_generate_messages_cpp _apriltags_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(apriltags_msgs_gencpp)
add_dependencies(apriltags_msgs_gencpp apriltags_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS apriltags_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(apriltags_msgs
  "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTag.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/apriltags_msgs
)
_generate_msg_eus(apriltags_msgs
  "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTagDetections.msg"
  "${MSG_I_FLAGS}"
  "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTag.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/apriltags_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(apriltags_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/apriltags_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(apriltags_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(apriltags_msgs_generate_messages apriltags_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTag.msg" NAME_WE)
add_dependencies(apriltags_msgs_generate_messages_eus _apriltags_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTagDetections.msg" NAME_WE)
add_dependencies(apriltags_msgs_generate_messages_eus _apriltags_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(apriltags_msgs_geneus)
add_dependencies(apriltags_msgs_geneus apriltags_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS apriltags_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(apriltags_msgs
  "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTag.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/apriltags_msgs
)
_generate_msg_lisp(apriltags_msgs
  "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTagDetections.msg"
  "${MSG_I_FLAGS}"
  "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTag.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/apriltags_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(apriltags_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/apriltags_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(apriltags_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(apriltags_msgs_generate_messages apriltags_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTag.msg" NAME_WE)
add_dependencies(apriltags_msgs_generate_messages_lisp _apriltags_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTagDetections.msg" NAME_WE)
add_dependencies(apriltags_msgs_generate_messages_lisp _apriltags_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(apriltags_msgs_genlisp)
add_dependencies(apriltags_msgs_genlisp apriltags_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS apriltags_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(apriltags_msgs
  "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTag.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/apriltags_msgs
)
_generate_msg_nodejs(apriltags_msgs
  "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTagDetections.msg"
  "${MSG_I_FLAGS}"
  "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTag.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/apriltags_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(apriltags_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/apriltags_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(apriltags_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(apriltags_msgs_generate_messages apriltags_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTag.msg" NAME_WE)
add_dependencies(apriltags_msgs_generate_messages_nodejs _apriltags_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTagDetections.msg" NAME_WE)
add_dependencies(apriltags_msgs_generate_messages_nodejs _apriltags_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(apriltags_msgs_gennodejs)
add_dependencies(apriltags_msgs_gennodejs apriltags_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS apriltags_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(apriltags_msgs
  "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTag.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltags_msgs
)
_generate_msg_py(apriltags_msgs
  "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTagDetections.msg"
  "${MSG_I_FLAGS}"
  "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTag.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltags_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(apriltags_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltags_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(apriltags_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(apriltags_msgs_generate_messages apriltags_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTag.msg" NAME_WE)
add_dependencies(apriltags_msgs_generate_messages_py _apriltags_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xinyu/workspace/360/apriltag_Dir/apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_msgs/msg/AprilTagDetections.msg" NAME_WE)
add_dependencies(apriltags_msgs_generate_messages_py _apriltags_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(apriltags_msgs_genpy)
add_dependencies(apriltags_msgs_genpy apriltags_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS apriltags_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/apriltags_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/apriltags_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(apriltags_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(apriltags_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/apriltags_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/apriltags_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(apriltags_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(apriltags_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/apriltags_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/apriltags_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(apriltags_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(apriltags_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/apriltags_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/apriltags_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(apriltags_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(apriltags_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltags_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltags_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltags_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(apriltags_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(apriltags_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
