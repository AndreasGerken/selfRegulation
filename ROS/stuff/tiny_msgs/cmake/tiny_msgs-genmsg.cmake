# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tiny_msgs: 2 messages, 0 services")

set(MSG_I_FLAGS "-Itiny_msgs:/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tiny_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyVector.msg" NAME_WE)
add_custom_target(_tiny_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiny_msgs" "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyVector.msg" ""
)

get_filename_component(_filename "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyIMU.msg" NAME_WE)
add_custom_target(_tiny_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiny_msgs" "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyIMU.msg" "std_msgs/Header:tiny_msgs/tinyVector"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tiny_msgs
  "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyVector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiny_msgs
)
_generate_msg_cpp(tiny_msgs
  "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyIMU.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyVector.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiny_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(tiny_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiny_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tiny_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tiny_msgs_generate_messages tiny_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyVector.msg" NAME_WE)
add_dependencies(tiny_msgs_generate_messages_cpp _tiny_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyIMU.msg" NAME_WE)
add_dependencies(tiny_msgs_generate_messages_cpp _tiny_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tiny_msgs_gencpp)
add_dependencies(tiny_msgs_gencpp tiny_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tiny_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tiny_msgs
  "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyVector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiny_msgs
)
_generate_msg_eus(tiny_msgs
  "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyIMU.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyVector.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiny_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(tiny_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiny_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tiny_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tiny_msgs_generate_messages tiny_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyVector.msg" NAME_WE)
add_dependencies(tiny_msgs_generate_messages_eus _tiny_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyIMU.msg" NAME_WE)
add_dependencies(tiny_msgs_generate_messages_eus _tiny_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tiny_msgs_geneus)
add_dependencies(tiny_msgs_geneus tiny_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tiny_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tiny_msgs
  "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyVector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiny_msgs
)
_generate_msg_lisp(tiny_msgs
  "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyIMU.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyVector.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiny_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(tiny_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiny_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tiny_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tiny_msgs_generate_messages tiny_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyVector.msg" NAME_WE)
add_dependencies(tiny_msgs_generate_messages_lisp _tiny_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyIMU.msg" NAME_WE)
add_dependencies(tiny_msgs_generate_messages_lisp _tiny_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tiny_msgs_genlisp)
add_dependencies(tiny_msgs_genlisp tiny_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tiny_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tiny_msgs
  "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyVector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiny_msgs
)
_generate_msg_nodejs(tiny_msgs
  "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyIMU.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyVector.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiny_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(tiny_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiny_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tiny_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tiny_msgs_generate_messages tiny_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyVector.msg" NAME_WE)
add_dependencies(tiny_msgs_generate_messages_nodejs _tiny_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyIMU.msg" NAME_WE)
add_dependencies(tiny_msgs_generate_messages_nodejs _tiny_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tiny_msgs_gennodejs)
add_dependencies(tiny_msgs_gennodejs tiny_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tiny_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tiny_msgs
  "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyVector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiny_msgs
)
_generate_msg_py(tiny_msgs
  "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyIMU.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyVector.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiny_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(tiny_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiny_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tiny_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tiny_msgs_generate_messages tiny_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyVector.msg" NAME_WE)
add_dependencies(tiny_msgs_generate_messages_py _tiny_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andi/Documents/selfRegulation/ROS/stuff/tiny_msgs/msg/tinyIMU.msg" NAME_WE)
add_dependencies(tiny_msgs_generate_messages_py _tiny_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tiny_msgs_genpy)
add_dependencies(tiny_msgs_genpy tiny_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tiny_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiny_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiny_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tiny_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiny_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiny_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tiny_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiny_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiny_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tiny_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiny_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiny_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tiny_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiny_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiny_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiny_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tiny_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
