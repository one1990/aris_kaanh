# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "controller: 3 messages, 2 services")

set(MSG_I_FLAGS "-Icontroller:/root/catkin_ws/src/controller/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(controller_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/catkin_ws/src/controller/srv/setpos.srv" NAME_WE)
add_custom_target(_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "controller" "/root/catkin_ws/src/controller/srv/setpos.srv" ""
)

get_filename_component(_filename "/root/catkin_ws/src/controller/srv/interface.srv" NAME_WE)
add_custom_target(_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "controller" "/root/catkin_ws/src/controller/srv/interface.srv" ""
)

get_filename_component(_filename "/root/catkin_ws/src/controller/msg/pose.msg" NAME_WE)
add_custom_target(_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "controller" "/root/catkin_ws/src/controller/msg/pose.msg" ""
)

get_filename_component(_filename "/root/catkin_ws/src/controller/msg/rob_param.msg" NAME_WE)
add_custom_target(_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "controller" "/root/catkin_ws/src/controller/msg/rob_param.msg" ""
)

get_filename_component(_filename "/root/catkin_ws/src/controller/msg/motorpos.msg" NAME_WE)
add_custom_target(_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "controller" "/root/catkin_ws/src/controller/msg/motorpos.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(controller
  "/root/catkin_ws/src/controller/msg/motorpos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller
)
_generate_msg_cpp(controller
  "/root/catkin_ws/src/controller/msg/rob_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller
)
_generate_msg_cpp(controller
  "/root/catkin_ws/src/controller/msg/pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller
)

### Generating Services
_generate_srv_cpp(controller
  "/root/catkin_ws/src/controller/srv/interface.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller
)
_generate_srv_cpp(controller
  "/root/catkin_ws/src/controller/srv/setpos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller
)

### Generating Module File
_generate_module_cpp(controller
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(controller_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(controller_generate_messages controller_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/controller/srv/setpos.srv" NAME_WE)
add_dependencies(controller_generate_messages_cpp _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/srv/interface.srv" NAME_WE)
add_dependencies(controller_generate_messages_cpp _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/msg/pose.msg" NAME_WE)
add_dependencies(controller_generate_messages_cpp _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/msg/rob_param.msg" NAME_WE)
add_dependencies(controller_generate_messages_cpp _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/msg/motorpos.msg" NAME_WE)
add_dependencies(controller_generate_messages_cpp _controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controller_gencpp)
add_dependencies(controller_gencpp controller_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controller_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(controller
  "/root/catkin_ws/src/controller/msg/motorpos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller
)
_generate_msg_eus(controller
  "/root/catkin_ws/src/controller/msg/rob_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller
)
_generate_msg_eus(controller
  "/root/catkin_ws/src/controller/msg/pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller
)

### Generating Services
_generate_srv_eus(controller
  "/root/catkin_ws/src/controller/srv/interface.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller
)
_generate_srv_eus(controller
  "/root/catkin_ws/src/controller/srv/setpos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller
)

### Generating Module File
_generate_module_eus(controller
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(controller_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(controller_generate_messages controller_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/controller/srv/setpos.srv" NAME_WE)
add_dependencies(controller_generate_messages_eus _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/srv/interface.srv" NAME_WE)
add_dependencies(controller_generate_messages_eus _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/msg/pose.msg" NAME_WE)
add_dependencies(controller_generate_messages_eus _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/msg/rob_param.msg" NAME_WE)
add_dependencies(controller_generate_messages_eus _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/msg/motorpos.msg" NAME_WE)
add_dependencies(controller_generate_messages_eus _controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controller_geneus)
add_dependencies(controller_geneus controller_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controller_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(controller
  "/root/catkin_ws/src/controller/msg/motorpos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller
)
_generate_msg_lisp(controller
  "/root/catkin_ws/src/controller/msg/rob_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller
)
_generate_msg_lisp(controller
  "/root/catkin_ws/src/controller/msg/pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller
)

### Generating Services
_generate_srv_lisp(controller
  "/root/catkin_ws/src/controller/srv/interface.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller
)
_generate_srv_lisp(controller
  "/root/catkin_ws/src/controller/srv/setpos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller
)

### Generating Module File
_generate_module_lisp(controller
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(controller_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(controller_generate_messages controller_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/controller/srv/setpos.srv" NAME_WE)
add_dependencies(controller_generate_messages_lisp _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/srv/interface.srv" NAME_WE)
add_dependencies(controller_generate_messages_lisp _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/msg/pose.msg" NAME_WE)
add_dependencies(controller_generate_messages_lisp _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/msg/rob_param.msg" NAME_WE)
add_dependencies(controller_generate_messages_lisp _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/msg/motorpos.msg" NAME_WE)
add_dependencies(controller_generate_messages_lisp _controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controller_genlisp)
add_dependencies(controller_genlisp controller_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controller_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(controller
  "/root/catkin_ws/src/controller/msg/motorpos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller
)
_generate_msg_nodejs(controller
  "/root/catkin_ws/src/controller/msg/rob_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller
)
_generate_msg_nodejs(controller
  "/root/catkin_ws/src/controller/msg/pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller
)

### Generating Services
_generate_srv_nodejs(controller
  "/root/catkin_ws/src/controller/srv/interface.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller
)
_generate_srv_nodejs(controller
  "/root/catkin_ws/src/controller/srv/setpos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller
)

### Generating Module File
_generate_module_nodejs(controller
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(controller_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(controller_generate_messages controller_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/controller/srv/setpos.srv" NAME_WE)
add_dependencies(controller_generate_messages_nodejs _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/srv/interface.srv" NAME_WE)
add_dependencies(controller_generate_messages_nodejs _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/msg/pose.msg" NAME_WE)
add_dependencies(controller_generate_messages_nodejs _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/msg/rob_param.msg" NAME_WE)
add_dependencies(controller_generate_messages_nodejs _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/msg/motorpos.msg" NAME_WE)
add_dependencies(controller_generate_messages_nodejs _controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controller_gennodejs)
add_dependencies(controller_gennodejs controller_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controller_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(controller
  "/root/catkin_ws/src/controller/msg/motorpos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller
)
_generate_msg_py(controller
  "/root/catkin_ws/src/controller/msg/rob_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller
)
_generate_msg_py(controller
  "/root/catkin_ws/src/controller/msg/pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller
)

### Generating Services
_generate_srv_py(controller
  "/root/catkin_ws/src/controller/srv/interface.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller
)
_generate_srv_py(controller
  "/root/catkin_ws/src/controller/srv/setpos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller
)

### Generating Module File
_generate_module_py(controller
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(controller_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(controller_generate_messages controller_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/controller/srv/setpos.srv" NAME_WE)
add_dependencies(controller_generate_messages_py _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/srv/interface.srv" NAME_WE)
add_dependencies(controller_generate_messages_py _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/msg/pose.msg" NAME_WE)
add_dependencies(controller_generate_messages_py _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/msg/rob_param.msg" NAME_WE)
add_dependencies(controller_generate_messages_py _controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/controller/msg/motorpos.msg" NAME_WE)
add_dependencies(controller_generate_messages_py _controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controller_genpy)
add_dependencies(controller_genpy controller_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controller_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(controller_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(controller_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(controller_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(controller_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(controller_generate_messages_py std_msgs_generate_messages_py)
endif()
