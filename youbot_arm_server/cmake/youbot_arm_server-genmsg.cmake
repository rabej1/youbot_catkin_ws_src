# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "youbot_arm_server: 7 messages, 0 services")

set(MSG_I_FLAGS "-Iyoubot_arm_server:/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(youbot_arm_server_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionGoal.msg" NAME_WE)
add_custom_target(_youbot_arm_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "youbot_arm_server" "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionGoal.msg" "geometry_msgs/Point:youbot_arm_server/YouBotGoalGoal:geometry_msgs/Quaternion:actionlib_msgs/GoalID:std_msgs/Header:geometry_msgs/PoseStamped:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalFeedback.msg" NAME_WE)
add_custom_target(_youbot_arm_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "youbot_arm_server" "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalFeedback.msg" "geometry_msgs/Point:geometry_msgs/PoseStamped:geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalResult.msg" NAME_WE)
add_custom_target(_youbot_arm_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "youbot_arm_server" "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalResult.msg" ""
)

get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionResult.msg" NAME_WE)
add_custom_target(_youbot_arm_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "youbot_arm_server" "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionResult.msg" "actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:std_msgs/Header:youbot_arm_server/YouBotGoalResult"
)

get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionFeedback.msg" NAME_WE)
add_custom_target(_youbot_arm_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "youbot_arm_server" "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionFeedback.msg" "geometry_msgs/Point:geometry_msgs/Quaternion:actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:youbot_arm_server/YouBotGoalFeedback:std_msgs/Header:geometry_msgs/PoseStamped:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalAction.msg" NAME_WE)
add_custom_target(_youbot_arm_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "youbot_arm_server" "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalAction.msg" "geometry_msgs/Point:youbot_arm_server/YouBotGoalGoal:geometry_msgs/Quaternion:actionlib_msgs/GoalStatus:youbot_arm_server/YouBotGoalActionResult:actionlib_msgs/GoalID:youbot_arm_server/YouBotGoalResult:youbot_arm_server/YouBotGoalFeedback:std_msgs/Header:youbot_arm_server/YouBotGoalActionGoal:geometry_msgs/PoseStamped:youbot_arm_server/YouBotGoalActionFeedback:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalGoal.msg" NAME_WE)
add_custom_target(_youbot_arm_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "youbot_arm_server" "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalGoal.msg" "geometry_msgs/Point:geometry_msgs/PoseStamped:geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Pose"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalGoal.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/youbot_arm_server
)
_generate_msg_cpp(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/youbot_arm_server
)
_generate_msg_cpp(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/youbot_arm_server
)
_generate_msg_cpp(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/youbot_arm_server
)
_generate_msg_cpp(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalFeedback.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/youbot_arm_server
)
_generate_msg_cpp(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalGoal.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalResult.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalFeedback.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionGoal.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionFeedback.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/youbot_arm_server
)
_generate_msg_cpp(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/youbot_arm_server
)

### Generating Services

### Generating Module File
_generate_module_cpp(youbot_arm_server
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/youbot_arm_server
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(youbot_arm_server_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(youbot_arm_server_generate_messages youbot_arm_server_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionGoal.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_cpp _youbot_arm_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalFeedback.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_cpp _youbot_arm_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalResult.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_cpp _youbot_arm_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionResult.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_cpp _youbot_arm_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionFeedback.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_cpp _youbot_arm_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalAction.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_cpp _youbot_arm_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalGoal.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_cpp _youbot_arm_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(youbot_arm_server_gencpp)
add_dependencies(youbot_arm_server_gencpp youbot_arm_server_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS youbot_arm_server_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalGoal.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/youbot_arm_server
)
_generate_msg_lisp(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/youbot_arm_server
)
_generate_msg_lisp(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/youbot_arm_server
)
_generate_msg_lisp(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/youbot_arm_server
)
_generate_msg_lisp(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalFeedback.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/youbot_arm_server
)
_generate_msg_lisp(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalGoal.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalResult.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalFeedback.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionGoal.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionFeedback.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/youbot_arm_server
)
_generate_msg_lisp(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/youbot_arm_server
)

### Generating Services

### Generating Module File
_generate_module_lisp(youbot_arm_server
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/youbot_arm_server
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(youbot_arm_server_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(youbot_arm_server_generate_messages youbot_arm_server_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionGoal.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_lisp _youbot_arm_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalFeedback.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_lisp _youbot_arm_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalResult.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_lisp _youbot_arm_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionResult.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_lisp _youbot_arm_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionFeedback.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_lisp _youbot_arm_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalAction.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_lisp _youbot_arm_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalGoal.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_lisp _youbot_arm_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(youbot_arm_server_genlisp)
add_dependencies(youbot_arm_server_genlisp youbot_arm_server_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS youbot_arm_server_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalGoal.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_arm_server
)
_generate_msg_py(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_arm_server
)
_generate_msg_py(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_arm_server
)
_generate_msg_py(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_arm_server
)
_generate_msg_py(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalFeedback.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_arm_server
)
_generate_msg_py(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalGoal.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalResult.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalFeedback.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionGoal.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionFeedback.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_arm_server
)
_generate_msg_py(youbot_arm_server
  "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_arm_server
)

### Generating Services

### Generating Module File
_generate_module_py(youbot_arm_server
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_arm_server
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(youbot_arm_server_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(youbot_arm_server_generate_messages youbot_arm_server_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionGoal.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_py _youbot_arm_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalFeedback.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_py _youbot_arm_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalResult.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_py _youbot_arm_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionResult.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_py _youbot_arm_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalActionFeedback.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_py _youbot_arm_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalAction.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_py _youbot_arm_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jonas/catkin_ws/devel/share/youbot_arm_server/msg/YouBotGoalGoal.msg" NAME_WE)
add_dependencies(youbot_arm_server_generate_messages_py _youbot_arm_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(youbot_arm_server_genpy)
add_dependencies(youbot_arm_server_genpy youbot_arm_server_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS youbot_arm_server_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/youbot_arm_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/youbot_arm_server
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(youbot_arm_server_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
add_dependencies(youbot_arm_server_generate_messages_cpp geometry_msgs_generate_messages_cpp)
add_dependencies(youbot_arm_server_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/youbot_arm_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/youbot_arm_server
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(youbot_arm_server_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
add_dependencies(youbot_arm_server_generate_messages_lisp geometry_msgs_generate_messages_lisp)
add_dependencies(youbot_arm_server_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_arm_server)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_arm_server\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_arm_server
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(youbot_arm_server_generate_messages_py actionlib_msgs_generate_messages_py)
add_dependencies(youbot_arm_server_generate_messages_py geometry_msgs_generate_messages_py)
add_dependencies(youbot_arm_server_generate_messages_py std_msgs_generate_messages_py)
