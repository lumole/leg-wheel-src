# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "message: 14 messages, 0 services")

set(MSG_I_FLAGS "-Imessage:/home/robot5/robot5_ws/src/message/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(message_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/cmb_interface.msg" NAME_WE)
add_custom_target(_message_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "message" "/home/robot5/robot5_ws/src/message/msg/cmb_interface.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/cmb_slave.msg" NAME_WE)
add_custom_target(_message_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "message" "/home/robot5/robot5_ws/src/message/msg/cmb_slave.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/imu_controller.msg" NAME_WE)
add_custom_target(_message_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "message" "/home/robot5/robot5_ws/src/message/msg/imu_controller.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/interface_controller.msg" NAME_WE)
add_custom_target(_message_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "message" "/home/robot5/robot5_ws/src/message/msg/interface_controller.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/pid_interface.msg" NAME_WE)
add_custom_target(_message_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "message" "/home/robot5/robot5_ws/src/message/msg/pid_interface.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/vmc_interface.msg" NAME_WE)
add_custom_target(_message_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "message" "/home/robot5/robot5_ws/src/message/msg/vmc_interface.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/kinematics_interface.msg" NAME_WE)
add_custom_target(_message_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "message" "/home/robot5/robot5_ws/src/message/msg/kinematics_interface.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/d435i_imu_controller.msg" NAME_WE)
add_custom_target(_message_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "message" "/home/robot5/robot5_ws/src/message/msg/d435i_imu_controller.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/lqr_state.msg" NAME_WE)
add_custom_target(_message_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "message" "/home/robot5/robot5_ws/src/message/msg/lqr_state.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/pid_state.msg" NAME_WE)
add_custom_target(_message_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "message" "/home/robot5/robot5_ws/src/message/msg/pid_state.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/manipulator.msg" NAME_WE)
add_custom_target(_message_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "message" "/home/robot5/robot5_ws/src/message/msg/manipulator.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/rs232.msg" NAME_WE)
add_custom_target(_message_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "message" "/home/robot5/robot5_ws/src/message/msg/rs232.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_ctrl.msg" NAME_WE)
add_custom_target(_message_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "message" "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_ctrl.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_state.msg" NAME_WE)
add_custom_target(_message_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "message" "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_state.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(message
  "/home/robot5/robot5_ws/src/message/msg/cmb_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/message
)
_generate_msg_cpp(message
  "/home/robot5/robot5_ws/src/message/msg/cmb_slave.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/message
)
_generate_msg_cpp(message
  "/home/robot5/robot5_ws/src/message/msg/imu_controller.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/message
)
_generate_msg_cpp(message
  "/home/robot5/robot5_ws/src/message/msg/interface_controller.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/message
)
_generate_msg_cpp(message
  "/home/robot5/robot5_ws/src/message/msg/pid_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/message
)
_generate_msg_cpp(message
  "/home/robot5/robot5_ws/src/message/msg/vmc_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/message
)
_generate_msg_cpp(message
  "/home/robot5/robot5_ws/src/message/msg/kinematics_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/message
)
_generate_msg_cpp(message
  "/home/robot5/robot5_ws/src/message/msg/d435i_imu_controller.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/message
)
_generate_msg_cpp(message
  "/home/robot5/robot5_ws/src/message/msg/lqr_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/message
)
_generate_msg_cpp(message
  "/home/robot5/robot5_ws/src/message/msg/pid_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/message
)
_generate_msg_cpp(message
  "/home/robot5/robot5_ws/src/message/msg/manipulator.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/message
)
_generate_msg_cpp(message
  "/home/robot5/robot5_ws/src/message/msg/rs232.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/message
)
_generate_msg_cpp(message
  "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_ctrl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/message
)
_generate_msg_cpp(message
  "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/message
)

### Generating Services

### Generating Module File
_generate_module_cpp(message
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/message
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(message_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(message_generate_messages message_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/cmb_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_cpp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/cmb_slave.msg" NAME_WE)
add_dependencies(message_generate_messages_cpp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/imu_controller.msg" NAME_WE)
add_dependencies(message_generate_messages_cpp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/interface_controller.msg" NAME_WE)
add_dependencies(message_generate_messages_cpp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/pid_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_cpp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/vmc_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_cpp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/kinematics_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_cpp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/d435i_imu_controller.msg" NAME_WE)
add_dependencies(message_generate_messages_cpp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/lqr_state.msg" NAME_WE)
add_dependencies(message_generate_messages_cpp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/pid_state.msg" NAME_WE)
add_dependencies(message_generate_messages_cpp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/manipulator.msg" NAME_WE)
add_dependencies(message_generate_messages_cpp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/rs232.msg" NAME_WE)
add_dependencies(message_generate_messages_cpp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_ctrl.msg" NAME_WE)
add_dependencies(message_generate_messages_cpp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_state.msg" NAME_WE)
add_dependencies(message_generate_messages_cpp _message_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(message_gencpp)
add_dependencies(message_gencpp message_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS message_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(message
  "/home/robot5/robot5_ws/src/message/msg/cmb_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/message
)
_generate_msg_eus(message
  "/home/robot5/robot5_ws/src/message/msg/cmb_slave.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/message
)
_generate_msg_eus(message
  "/home/robot5/robot5_ws/src/message/msg/imu_controller.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/message
)
_generate_msg_eus(message
  "/home/robot5/robot5_ws/src/message/msg/interface_controller.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/message
)
_generate_msg_eus(message
  "/home/robot5/robot5_ws/src/message/msg/pid_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/message
)
_generate_msg_eus(message
  "/home/robot5/robot5_ws/src/message/msg/vmc_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/message
)
_generate_msg_eus(message
  "/home/robot5/robot5_ws/src/message/msg/kinematics_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/message
)
_generate_msg_eus(message
  "/home/robot5/robot5_ws/src/message/msg/d435i_imu_controller.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/message
)
_generate_msg_eus(message
  "/home/robot5/robot5_ws/src/message/msg/lqr_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/message
)
_generate_msg_eus(message
  "/home/robot5/robot5_ws/src/message/msg/pid_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/message
)
_generate_msg_eus(message
  "/home/robot5/robot5_ws/src/message/msg/manipulator.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/message
)
_generate_msg_eus(message
  "/home/robot5/robot5_ws/src/message/msg/rs232.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/message
)
_generate_msg_eus(message
  "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_ctrl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/message
)
_generate_msg_eus(message
  "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/message
)

### Generating Services

### Generating Module File
_generate_module_eus(message
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/message
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(message_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(message_generate_messages message_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/cmb_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_eus _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/cmb_slave.msg" NAME_WE)
add_dependencies(message_generate_messages_eus _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/imu_controller.msg" NAME_WE)
add_dependencies(message_generate_messages_eus _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/interface_controller.msg" NAME_WE)
add_dependencies(message_generate_messages_eus _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/pid_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_eus _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/vmc_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_eus _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/kinematics_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_eus _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/d435i_imu_controller.msg" NAME_WE)
add_dependencies(message_generate_messages_eus _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/lqr_state.msg" NAME_WE)
add_dependencies(message_generate_messages_eus _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/pid_state.msg" NAME_WE)
add_dependencies(message_generate_messages_eus _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/manipulator.msg" NAME_WE)
add_dependencies(message_generate_messages_eus _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/rs232.msg" NAME_WE)
add_dependencies(message_generate_messages_eus _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_ctrl.msg" NAME_WE)
add_dependencies(message_generate_messages_eus _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_state.msg" NAME_WE)
add_dependencies(message_generate_messages_eus _message_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(message_geneus)
add_dependencies(message_geneus message_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS message_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(message
  "/home/robot5/robot5_ws/src/message/msg/cmb_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/message
)
_generate_msg_lisp(message
  "/home/robot5/robot5_ws/src/message/msg/cmb_slave.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/message
)
_generate_msg_lisp(message
  "/home/robot5/robot5_ws/src/message/msg/imu_controller.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/message
)
_generate_msg_lisp(message
  "/home/robot5/robot5_ws/src/message/msg/interface_controller.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/message
)
_generate_msg_lisp(message
  "/home/robot5/robot5_ws/src/message/msg/pid_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/message
)
_generate_msg_lisp(message
  "/home/robot5/robot5_ws/src/message/msg/vmc_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/message
)
_generate_msg_lisp(message
  "/home/robot5/robot5_ws/src/message/msg/kinematics_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/message
)
_generate_msg_lisp(message
  "/home/robot5/robot5_ws/src/message/msg/d435i_imu_controller.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/message
)
_generate_msg_lisp(message
  "/home/robot5/robot5_ws/src/message/msg/lqr_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/message
)
_generate_msg_lisp(message
  "/home/robot5/robot5_ws/src/message/msg/pid_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/message
)
_generate_msg_lisp(message
  "/home/robot5/robot5_ws/src/message/msg/manipulator.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/message
)
_generate_msg_lisp(message
  "/home/robot5/robot5_ws/src/message/msg/rs232.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/message
)
_generate_msg_lisp(message
  "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_ctrl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/message
)
_generate_msg_lisp(message
  "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/message
)

### Generating Services

### Generating Module File
_generate_module_lisp(message
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/message
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(message_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(message_generate_messages message_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/cmb_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_lisp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/cmb_slave.msg" NAME_WE)
add_dependencies(message_generate_messages_lisp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/imu_controller.msg" NAME_WE)
add_dependencies(message_generate_messages_lisp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/interface_controller.msg" NAME_WE)
add_dependencies(message_generate_messages_lisp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/pid_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_lisp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/vmc_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_lisp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/kinematics_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_lisp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/d435i_imu_controller.msg" NAME_WE)
add_dependencies(message_generate_messages_lisp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/lqr_state.msg" NAME_WE)
add_dependencies(message_generate_messages_lisp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/pid_state.msg" NAME_WE)
add_dependencies(message_generate_messages_lisp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/manipulator.msg" NAME_WE)
add_dependencies(message_generate_messages_lisp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/rs232.msg" NAME_WE)
add_dependencies(message_generate_messages_lisp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_ctrl.msg" NAME_WE)
add_dependencies(message_generate_messages_lisp _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_state.msg" NAME_WE)
add_dependencies(message_generate_messages_lisp _message_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(message_genlisp)
add_dependencies(message_genlisp message_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS message_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(message
  "/home/robot5/robot5_ws/src/message/msg/cmb_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/message
)
_generate_msg_nodejs(message
  "/home/robot5/robot5_ws/src/message/msg/cmb_slave.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/message
)
_generate_msg_nodejs(message
  "/home/robot5/robot5_ws/src/message/msg/imu_controller.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/message
)
_generate_msg_nodejs(message
  "/home/robot5/robot5_ws/src/message/msg/interface_controller.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/message
)
_generate_msg_nodejs(message
  "/home/robot5/robot5_ws/src/message/msg/pid_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/message
)
_generate_msg_nodejs(message
  "/home/robot5/robot5_ws/src/message/msg/vmc_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/message
)
_generate_msg_nodejs(message
  "/home/robot5/robot5_ws/src/message/msg/kinematics_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/message
)
_generate_msg_nodejs(message
  "/home/robot5/robot5_ws/src/message/msg/d435i_imu_controller.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/message
)
_generate_msg_nodejs(message
  "/home/robot5/robot5_ws/src/message/msg/lqr_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/message
)
_generate_msg_nodejs(message
  "/home/robot5/robot5_ws/src/message/msg/pid_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/message
)
_generate_msg_nodejs(message
  "/home/robot5/robot5_ws/src/message/msg/manipulator.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/message
)
_generate_msg_nodejs(message
  "/home/robot5/robot5_ws/src/message/msg/rs232.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/message
)
_generate_msg_nodejs(message
  "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_ctrl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/message
)
_generate_msg_nodejs(message
  "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/message
)

### Generating Services

### Generating Module File
_generate_module_nodejs(message
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/message
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(message_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(message_generate_messages message_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/cmb_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_nodejs _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/cmb_slave.msg" NAME_WE)
add_dependencies(message_generate_messages_nodejs _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/imu_controller.msg" NAME_WE)
add_dependencies(message_generate_messages_nodejs _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/interface_controller.msg" NAME_WE)
add_dependencies(message_generate_messages_nodejs _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/pid_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_nodejs _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/vmc_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_nodejs _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/kinematics_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_nodejs _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/d435i_imu_controller.msg" NAME_WE)
add_dependencies(message_generate_messages_nodejs _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/lqr_state.msg" NAME_WE)
add_dependencies(message_generate_messages_nodejs _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/pid_state.msg" NAME_WE)
add_dependencies(message_generate_messages_nodejs _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/manipulator.msg" NAME_WE)
add_dependencies(message_generate_messages_nodejs _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/rs232.msg" NAME_WE)
add_dependencies(message_generate_messages_nodejs _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_ctrl.msg" NAME_WE)
add_dependencies(message_generate_messages_nodejs _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_state.msg" NAME_WE)
add_dependencies(message_generate_messages_nodejs _message_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(message_gennodejs)
add_dependencies(message_gennodejs message_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS message_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(message
  "/home/robot5/robot5_ws/src/message/msg/cmb_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/message
)
_generate_msg_py(message
  "/home/robot5/robot5_ws/src/message/msg/cmb_slave.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/message
)
_generate_msg_py(message
  "/home/robot5/robot5_ws/src/message/msg/imu_controller.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/message
)
_generate_msg_py(message
  "/home/robot5/robot5_ws/src/message/msg/interface_controller.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/message
)
_generate_msg_py(message
  "/home/robot5/robot5_ws/src/message/msg/pid_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/message
)
_generate_msg_py(message
  "/home/robot5/robot5_ws/src/message/msg/vmc_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/message
)
_generate_msg_py(message
  "/home/robot5/robot5_ws/src/message/msg/kinematics_interface.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/message
)
_generate_msg_py(message
  "/home/robot5/robot5_ws/src/message/msg/d435i_imu_controller.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/message
)
_generate_msg_py(message
  "/home/robot5/robot5_ws/src/message/msg/lqr_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/message
)
_generate_msg_py(message
  "/home/robot5/robot5_ws/src/message/msg/pid_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/message
)
_generate_msg_py(message
  "/home/robot5/robot5_ws/src/message/msg/manipulator.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/message
)
_generate_msg_py(message
  "/home/robot5/robot5_ws/src/message/msg/rs232.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/message
)
_generate_msg_py(message
  "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_ctrl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/message
)
_generate_msg_py(message
  "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/message
)

### Generating Services

### Generating Module File
_generate_module_py(message
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/message
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(message_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(message_generate_messages message_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/cmb_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_py _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/cmb_slave.msg" NAME_WE)
add_dependencies(message_generate_messages_py _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/imu_controller.msg" NAME_WE)
add_dependencies(message_generate_messages_py _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/interface_controller.msg" NAME_WE)
add_dependencies(message_generate_messages_py _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/pid_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_py _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/vmc_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_py _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/kinematics_interface.msg" NAME_WE)
add_dependencies(message_generate_messages_py _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/d435i_imu_controller.msg" NAME_WE)
add_dependencies(message_generate_messages_py _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/lqr_state.msg" NAME_WE)
add_dependencies(message_generate_messages_py _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/pid_state.msg" NAME_WE)
add_dependencies(message_generate_messages_py _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/manipulator.msg" NAME_WE)
add_dependencies(message_generate_messages_py _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/rs232.msg" NAME_WE)
add_dependencies(message_generate_messages_py _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_ctrl.msg" NAME_WE)
add_dependencies(message_generate_messages_py _message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot5/robot5_ws/src/message/msg/rs232_elec_mag_state.msg" NAME_WE)
add_dependencies(message_generate_messages_py _message_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(message_genpy)
add_dependencies(message_genpy message_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS message_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/message)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/message
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(message_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/message)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/message
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(message_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/message)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/message
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(message_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/message)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/message
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(message_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/message)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/message\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/message
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(message_generate_messages_py std_msgs_generate_messages_py)
endif()
