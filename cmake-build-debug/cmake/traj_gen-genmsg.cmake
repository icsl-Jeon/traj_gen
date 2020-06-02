# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "traj_gen: 3 messages, 0 services")

set(MSG_I_FLAGS "-Itraj_gen:/home/jbs/test_ws/src/traj_gen/msg;-Itraj_gen:/home/jbs/test_ws/src/traj_gen/msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(traj_gen_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg" NAME_WE)
add_custom_target(_traj_gen_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "traj_gen" "/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg" ""
)

get_filename_component(_filename "/home/jbs/test_ws/src/traj_gen/msg/PolySpline.msg" NAME_WE)
add_custom_target(_traj_gen_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "traj_gen" "/home/jbs/test_ws/src/traj_gen/msg/PolySpline.msg" "traj_gen/PolyCoeff"
)

get_filename_component(_filename "/home/jbs/test_ws/src/traj_gen/msg/PolySplineXYZ.msg" NAME_WE)
add_custom_target(_traj_gen_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "traj_gen" "/home/jbs/test_ws/src/traj_gen/msg/PolySplineXYZ.msg" "traj_gen/PolySpline:traj_gen/PolyCoeff"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(traj_gen
  "/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traj_gen
)
_generate_msg_cpp(traj_gen
  "/home/jbs/test_ws/src/traj_gen/msg/PolySplineXYZ.msg"
  "${MSG_I_FLAGS}"
  "/home/jbs/test_ws/src/traj_gen/msg/PolySpline.msg;/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traj_gen
)
_generate_msg_cpp(traj_gen
  "/home/jbs/test_ws/src/traj_gen/msg/PolySpline.msg"
  "${MSG_I_FLAGS}"
  "/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traj_gen
)

### Generating Services

### Generating Module File
_generate_module_cpp(traj_gen
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traj_gen
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(traj_gen_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(traj_gen_generate_messages traj_gen_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg" NAME_WE)
add_dependencies(traj_gen_generate_messages_cpp _traj_gen_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jbs/test_ws/src/traj_gen/msg/PolySpline.msg" NAME_WE)
add_dependencies(traj_gen_generate_messages_cpp _traj_gen_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jbs/test_ws/src/traj_gen/msg/PolySplineXYZ.msg" NAME_WE)
add_dependencies(traj_gen_generate_messages_cpp _traj_gen_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(traj_gen_gencpp)
add_dependencies(traj_gen_gencpp traj_gen_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS traj_gen_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(traj_gen
  "/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/traj_gen
)
_generate_msg_eus(traj_gen
  "/home/jbs/test_ws/src/traj_gen/msg/PolySplineXYZ.msg"
  "${MSG_I_FLAGS}"
  "/home/jbs/test_ws/src/traj_gen/msg/PolySpline.msg;/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/traj_gen
)
_generate_msg_eus(traj_gen
  "/home/jbs/test_ws/src/traj_gen/msg/PolySpline.msg"
  "${MSG_I_FLAGS}"
  "/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/traj_gen
)

### Generating Services

### Generating Module File
_generate_module_eus(traj_gen
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/traj_gen
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(traj_gen_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(traj_gen_generate_messages traj_gen_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg" NAME_WE)
add_dependencies(traj_gen_generate_messages_eus _traj_gen_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jbs/test_ws/src/traj_gen/msg/PolySpline.msg" NAME_WE)
add_dependencies(traj_gen_generate_messages_eus _traj_gen_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jbs/test_ws/src/traj_gen/msg/PolySplineXYZ.msg" NAME_WE)
add_dependencies(traj_gen_generate_messages_eus _traj_gen_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(traj_gen_geneus)
add_dependencies(traj_gen_geneus traj_gen_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS traj_gen_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(traj_gen
  "/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traj_gen
)
_generate_msg_lisp(traj_gen
  "/home/jbs/test_ws/src/traj_gen/msg/PolySplineXYZ.msg"
  "${MSG_I_FLAGS}"
  "/home/jbs/test_ws/src/traj_gen/msg/PolySpline.msg;/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traj_gen
)
_generate_msg_lisp(traj_gen
  "/home/jbs/test_ws/src/traj_gen/msg/PolySpline.msg"
  "${MSG_I_FLAGS}"
  "/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traj_gen
)

### Generating Services

### Generating Module File
_generate_module_lisp(traj_gen
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traj_gen
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(traj_gen_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(traj_gen_generate_messages traj_gen_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg" NAME_WE)
add_dependencies(traj_gen_generate_messages_lisp _traj_gen_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jbs/test_ws/src/traj_gen/msg/PolySpline.msg" NAME_WE)
add_dependencies(traj_gen_generate_messages_lisp _traj_gen_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jbs/test_ws/src/traj_gen/msg/PolySplineXYZ.msg" NAME_WE)
add_dependencies(traj_gen_generate_messages_lisp _traj_gen_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(traj_gen_genlisp)
add_dependencies(traj_gen_genlisp traj_gen_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS traj_gen_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(traj_gen
  "/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/traj_gen
)
_generate_msg_nodejs(traj_gen
  "/home/jbs/test_ws/src/traj_gen/msg/PolySplineXYZ.msg"
  "${MSG_I_FLAGS}"
  "/home/jbs/test_ws/src/traj_gen/msg/PolySpline.msg;/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/traj_gen
)
_generate_msg_nodejs(traj_gen
  "/home/jbs/test_ws/src/traj_gen/msg/PolySpline.msg"
  "${MSG_I_FLAGS}"
  "/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/traj_gen
)

### Generating Services

### Generating Module File
_generate_module_nodejs(traj_gen
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/traj_gen
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(traj_gen_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(traj_gen_generate_messages traj_gen_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg" NAME_WE)
add_dependencies(traj_gen_generate_messages_nodejs _traj_gen_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jbs/test_ws/src/traj_gen/msg/PolySpline.msg" NAME_WE)
add_dependencies(traj_gen_generate_messages_nodejs _traj_gen_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jbs/test_ws/src/traj_gen/msg/PolySplineXYZ.msg" NAME_WE)
add_dependencies(traj_gen_generate_messages_nodejs _traj_gen_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(traj_gen_gennodejs)
add_dependencies(traj_gen_gennodejs traj_gen_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS traj_gen_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(traj_gen
  "/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traj_gen
)
_generate_msg_py(traj_gen
  "/home/jbs/test_ws/src/traj_gen/msg/PolySplineXYZ.msg"
  "${MSG_I_FLAGS}"
  "/home/jbs/test_ws/src/traj_gen/msg/PolySpline.msg;/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traj_gen
)
_generate_msg_py(traj_gen
  "/home/jbs/test_ws/src/traj_gen/msg/PolySpline.msg"
  "${MSG_I_FLAGS}"
  "/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traj_gen
)

### Generating Services

### Generating Module File
_generate_module_py(traj_gen
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traj_gen
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(traj_gen_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(traj_gen_generate_messages traj_gen_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jbs/test_ws/src/traj_gen/msg/PolyCoeff.msg" NAME_WE)
add_dependencies(traj_gen_generate_messages_py _traj_gen_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jbs/test_ws/src/traj_gen/msg/PolySpline.msg" NAME_WE)
add_dependencies(traj_gen_generate_messages_py _traj_gen_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jbs/test_ws/src/traj_gen/msg/PolySplineXYZ.msg" NAME_WE)
add_dependencies(traj_gen_generate_messages_py _traj_gen_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(traj_gen_genpy)
add_dependencies(traj_gen_genpy traj_gen_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS traj_gen_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traj_gen)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traj_gen
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET traj_gen_generate_messages_cpp)
  add_dependencies(traj_gen_generate_messages_cpp traj_gen_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(traj_gen_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(traj_gen_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/traj_gen)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/traj_gen
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET traj_gen_generate_messages_eus)
  add_dependencies(traj_gen_generate_messages_eus traj_gen_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(traj_gen_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(traj_gen_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traj_gen)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traj_gen
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET traj_gen_generate_messages_lisp)
  add_dependencies(traj_gen_generate_messages_lisp traj_gen_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(traj_gen_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(traj_gen_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/traj_gen)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/traj_gen
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET traj_gen_generate_messages_nodejs)
  add_dependencies(traj_gen_generate_messages_nodejs traj_gen_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(traj_gen_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(traj_gen_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traj_gen)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traj_gen\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traj_gen
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET traj_gen_generate_messages_py)
  add_dependencies(traj_gen_generate_messages_py traj_gen_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(traj_gen_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(traj_gen_generate_messages_py std_msgs_generate_messages_py)
endif()
