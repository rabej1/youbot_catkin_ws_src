# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jonas/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jonas/catkin_ws/src

# Include any dependencies generated for this target.
include youbot_arm_server/CMakeFiles/youbot_test_client.dir/depend.make

# Include the progress variables for this target.
include youbot_arm_server/CMakeFiles/youbot_test_client.dir/progress.make

# Include the compile flags for this target's objects.
include youbot_arm_server/CMakeFiles/youbot_test_client.dir/flags.make

youbot_arm_server/CMakeFiles/youbot_test_client.dir/src/test_server.cpp.o: youbot_arm_server/CMakeFiles/youbot_test_client.dir/flags.make
youbot_arm_server/CMakeFiles/youbot_test_client.dir/src/test_server.cpp.o: youbot_arm_server/src/test_server.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/jonas/catkin_ws/src/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object youbot_arm_server/CMakeFiles/youbot_test_client.dir/src/test_server.cpp.o"
	cd /home/jonas/catkin_ws/src/youbot_arm_server && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/youbot_test_client.dir/src/test_server.cpp.o -c /home/jonas/catkin_ws/src/youbot_arm_server/src/test_server.cpp

youbot_arm_server/CMakeFiles/youbot_test_client.dir/src/test_server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/youbot_test_client.dir/src/test_server.cpp.i"
	cd /home/jonas/catkin_ws/src/youbot_arm_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/jonas/catkin_ws/src/youbot_arm_server/src/test_server.cpp > CMakeFiles/youbot_test_client.dir/src/test_server.cpp.i

youbot_arm_server/CMakeFiles/youbot_test_client.dir/src/test_server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/youbot_test_client.dir/src/test_server.cpp.s"
	cd /home/jonas/catkin_ws/src/youbot_arm_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/jonas/catkin_ws/src/youbot_arm_server/src/test_server.cpp -o CMakeFiles/youbot_test_client.dir/src/test_server.cpp.s

youbot_arm_server/CMakeFiles/youbot_test_client.dir/src/test_server.cpp.o.requires:
.PHONY : youbot_arm_server/CMakeFiles/youbot_test_client.dir/src/test_server.cpp.o.requires

youbot_arm_server/CMakeFiles/youbot_test_client.dir/src/test_server.cpp.o.provides: youbot_arm_server/CMakeFiles/youbot_test_client.dir/src/test_server.cpp.o.requires
	$(MAKE) -f youbot_arm_server/CMakeFiles/youbot_test_client.dir/build.make youbot_arm_server/CMakeFiles/youbot_test_client.dir/src/test_server.cpp.o.provides.build
.PHONY : youbot_arm_server/CMakeFiles/youbot_test_client.dir/src/test_server.cpp.o.provides

youbot_arm_server/CMakeFiles/youbot_test_client.dir/src/test_server.cpp.o.provides.build: youbot_arm_server/CMakeFiles/youbot_test_client.dir/src/test_server.cpp.o

# Object files for target youbot_test_client
youbot_test_client_OBJECTS = \
"CMakeFiles/youbot_test_client.dir/src/test_server.cpp.o"

# External object files for target youbot_test_client
youbot_test_client_EXTERNAL_OBJECTS =

/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: youbot_arm_server/CMakeFiles/youbot_test_client.dir/src/test_server.cpp.o
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: youbot_arm_server/CMakeFiles/youbot_test_client.dir/build.make
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_common_planning_interface_objects.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_planning_scene_interface.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_move_group_interface.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_warehouse.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libwarehouse_ros.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_pick_place_planner.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_move_group_capabilities_base.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_rdf_loader.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_kinematics_plugin_loader.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_robot_model_loader.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_constraint_sampler_manager_loader.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_planning_pipeline.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_trajectory_execution_manager.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_plan_execution.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_planning_scene_monitor.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_lazy_free_space_updater.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_point_containment_filter.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_occupancy_map_monitor.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_pointcloud_octomap_updater_core.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_semantic_world.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_exceptions.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_background_processing.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_kinematics_base.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_robot_model.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_transforms.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_robot_state.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_robot_trajectory.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_planning_interface.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_collision_detection.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_collision_detection_fcl.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_kinematic_constraints.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_planning_scene.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_constraint_samplers.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_planning_request_adapter.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_profiler.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_trajectory_processing.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_distance_field.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_kinematics_metrics.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmoveit_dynamics_solver.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libgeometric_shapes.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/liboctomap.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/liboctomath.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libeigen_conversions.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/librandom_numbers.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libkdl_parser.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/liborocos-kdl.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/liburdf.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/librosconsole_bridge.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libsrdfdom.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libimage_transport.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libclass_loader.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /usr/lib/libPocoFoundation.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /usr/lib/x86_64-linux-gnu/libdl.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libroslib.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libtf_conversions.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libkdl_conversions.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/liborocos-kdl.so.1.3.0
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libtf.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libtf2_ros.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libactionlib.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libmessage_filters.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libroscpp.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libtf2.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/librosconsole.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /usr/lib/liblog4cxx.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/librostime.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /opt/ros/indigo/lib/libcpp_common.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client: youbot_arm_server/CMakeFiles/youbot_test_client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client"
	cd /home/jonas/catkin_ws/src/youbot_arm_server && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/youbot_test_client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
youbot_arm_server/CMakeFiles/youbot_test_client.dir/build: /home/jonas/catkin_ws/devel/lib/youbot_arm_server/youbot_test_client
.PHONY : youbot_arm_server/CMakeFiles/youbot_test_client.dir/build

youbot_arm_server/CMakeFiles/youbot_test_client.dir/requires: youbot_arm_server/CMakeFiles/youbot_test_client.dir/src/test_server.cpp.o.requires
.PHONY : youbot_arm_server/CMakeFiles/youbot_test_client.dir/requires

youbot_arm_server/CMakeFiles/youbot_test_client.dir/clean:
	cd /home/jonas/catkin_ws/src/youbot_arm_server && $(CMAKE_COMMAND) -P CMakeFiles/youbot_test_client.dir/cmake_clean.cmake
.PHONY : youbot_arm_server/CMakeFiles/youbot_test_client.dir/clean

youbot_arm_server/CMakeFiles/youbot_test_client.dir/depend:
	cd /home/jonas/catkin_ws/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jonas/catkin_ws/src /home/jonas/catkin_ws/src/youbot_arm_server /home/jonas/catkin_ws/src /home/jonas/catkin_ws/src/youbot_arm_server /home/jonas/catkin_ws/src/youbot_arm_server/CMakeFiles/youbot_test_client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : youbot_arm_server/CMakeFiles/youbot_test_client.dir/depend

