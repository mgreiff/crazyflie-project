# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "crazy_ros: 2 messages, 0 services")

set(MSG_I_FLAGS "-Icrazy_ros:/home/daniel/Documents/Crazyflie/crazyflie_project/ROS/crazyflie_ws/src/crazy_ros/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(crazy_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/daniel/Documents/Crazyflie/crazyflie_project/ROS/crazyflie_ws/src/crazy_ros/msg/NumpyArrayFloat64.msg" NAME_WE)
add_custom_target(_crazy_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "crazy_ros" "/home/daniel/Documents/Crazyflie/crazyflie_project/ROS/crazyflie_ws/src/crazy_ros/msg/NumpyArrayFloat64.msg" ""
)

get_filename_component(_filename "/home/daniel/Documents/Crazyflie/crazyflie_project/ROS/crazyflie_ws/src/crazy_ros/msg/Num.msg" NAME_WE)
add_custom_target(_crazy_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "crazy_ros" "/home/daniel/Documents/Crazyflie/crazyflie_project/ROS/crazyflie_ws/src/crazy_ros/msg/Num.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(crazy_ros
  "/home/daniel/Documents/Crazyflie/crazyflie_project/ROS/crazyflie_ws/src/crazy_ros/msg/NumpyArrayFloat64.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crazy_ros
)
_generate_msg_cpp(crazy_ros
  "/home/daniel/Documents/Crazyflie/crazyflie_project/ROS/crazyflie_ws/src/crazy_ros/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crazy_ros
)

### Generating Services

### Generating Module File
_generate_module_cpp(crazy_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crazy_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(crazy_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(crazy_ros_generate_messages crazy_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/daniel/Documents/Crazyflie/crazyflie_project/ROS/crazyflie_ws/src/crazy_ros/msg/NumpyArrayFloat64.msg" NAME_WE)
add_dependencies(crazy_ros_generate_messages_cpp _crazy_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/daniel/Documents/Crazyflie/crazyflie_project/ROS/crazyflie_ws/src/crazy_ros/msg/Num.msg" NAME_WE)
add_dependencies(crazy_ros_generate_messages_cpp _crazy_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(crazy_ros_gencpp)
add_dependencies(crazy_ros_gencpp crazy_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS crazy_ros_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(crazy_ros
  "/home/daniel/Documents/Crazyflie/crazyflie_project/ROS/crazyflie_ws/src/crazy_ros/msg/NumpyArrayFloat64.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crazy_ros
)
_generate_msg_lisp(crazy_ros
  "/home/daniel/Documents/Crazyflie/crazyflie_project/ROS/crazyflie_ws/src/crazy_ros/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crazy_ros
)

### Generating Services

### Generating Module File
_generate_module_lisp(crazy_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crazy_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(crazy_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(crazy_ros_generate_messages crazy_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/daniel/Documents/Crazyflie/crazyflie_project/ROS/crazyflie_ws/src/crazy_ros/msg/NumpyArrayFloat64.msg" NAME_WE)
add_dependencies(crazy_ros_generate_messages_lisp _crazy_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/daniel/Documents/Crazyflie/crazyflie_project/ROS/crazyflie_ws/src/crazy_ros/msg/Num.msg" NAME_WE)
add_dependencies(crazy_ros_generate_messages_lisp _crazy_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(crazy_ros_genlisp)
add_dependencies(crazy_ros_genlisp crazy_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS crazy_ros_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(crazy_ros
  "/home/daniel/Documents/Crazyflie/crazyflie_project/ROS/crazyflie_ws/src/crazy_ros/msg/NumpyArrayFloat64.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crazy_ros
)
_generate_msg_py(crazy_ros
  "/home/daniel/Documents/Crazyflie/crazyflie_project/ROS/crazyflie_ws/src/crazy_ros/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crazy_ros
)

### Generating Services

### Generating Module File
_generate_module_py(crazy_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crazy_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(crazy_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(crazy_ros_generate_messages crazy_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/daniel/Documents/Crazyflie/crazyflie_project/ROS/crazyflie_ws/src/crazy_ros/msg/NumpyArrayFloat64.msg" NAME_WE)
add_dependencies(crazy_ros_generate_messages_py _crazy_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/daniel/Documents/Crazyflie/crazyflie_project/ROS/crazyflie_ws/src/crazy_ros/msg/Num.msg" NAME_WE)
add_dependencies(crazy_ros_generate_messages_py _crazy_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(crazy_ros_genpy)
add_dependencies(crazy_ros_genpy crazy_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS crazy_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crazy_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crazy_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(crazy_ros_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crazy_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crazy_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(crazy_ros_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crazy_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crazy_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crazy_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(crazy_ros_generate_messages_py std_msgs_generate_messages_py)
