# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/robot5/robot5_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robot5/robot5_ws/src/build

# Include any dependencies generated for this target.
include rs232_pkg/CMakeFiles/elec_mag.dir/depend.make

# Include the progress variables for this target.
include rs232_pkg/CMakeFiles/elec_mag.dir/progress.make

# Include the compile flags for this target's objects.
include rs232_pkg/CMakeFiles/elec_mag.dir/flags.make

rs232_pkg/CMakeFiles/elec_mag.dir/src/elec_mag.cpp.o: rs232_pkg/CMakeFiles/elec_mag.dir/flags.make
rs232_pkg/CMakeFiles/elec_mag.dir/src/elec_mag.cpp.o: ../rs232_pkg/src/elec_mag.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot5/robot5_ws/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object rs232_pkg/CMakeFiles/elec_mag.dir/src/elec_mag.cpp.o"
	cd /home/robot5/robot5_ws/src/build/rs232_pkg && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/elec_mag.dir/src/elec_mag.cpp.o -c /home/robot5/robot5_ws/src/rs232_pkg/src/elec_mag.cpp

rs232_pkg/CMakeFiles/elec_mag.dir/src/elec_mag.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/elec_mag.dir/src/elec_mag.cpp.i"
	cd /home/robot5/robot5_ws/src/build/rs232_pkg && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robot5/robot5_ws/src/rs232_pkg/src/elec_mag.cpp > CMakeFiles/elec_mag.dir/src/elec_mag.cpp.i

rs232_pkg/CMakeFiles/elec_mag.dir/src/elec_mag.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/elec_mag.dir/src/elec_mag.cpp.s"
	cd /home/robot5/robot5_ws/src/build/rs232_pkg && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robot5/robot5_ws/src/rs232_pkg/src/elec_mag.cpp -o CMakeFiles/elec_mag.dir/src/elec_mag.cpp.s

# Object files for target elec_mag
elec_mag_OBJECTS = \
"CMakeFiles/elec_mag.dir/src/elec_mag.cpp.o"

# External object files for target elec_mag
elec_mag_EXTERNAL_OBJECTS =

devel/lib/rs232_pkg/elec_mag: rs232_pkg/CMakeFiles/elec_mag.dir/src/elec_mag.cpp.o
devel/lib/rs232_pkg/elec_mag: rs232_pkg/CMakeFiles/elec_mag.dir/build.make
devel/lib/rs232_pkg/elec_mag: /opt/ros/noetic/lib/libserial.so
devel/lib/rs232_pkg/elec_mag: /opt/ros/noetic/lib/libroscpp.so
devel/lib/rs232_pkg/elec_mag: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/rs232_pkg/elec_mag: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
devel/lib/rs232_pkg/elec_mag: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
devel/lib/rs232_pkg/elec_mag: /opt/ros/noetic/lib/librosconsole.so
devel/lib/rs232_pkg/elec_mag: /opt/ros/noetic/lib/librosconsole_log4cxx.so
devel/lib/rs232_pkg/elec_mag: /opt/ros/noetic/lib/librosconsole_backend_interface.so
devel/lib/rs232_pkg/elec_mag: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/rs232_pkg/elec_mag: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
devel/lib/rs232_pkg/elec_mag: /opt/ros/noetic/lib/libxmlrpcpp.so
devel/lib/rs232_pkg/elec_mag: /opt/ros/noetic/lib/libroscpp_serialization.so
devel/lib/rs232_pkg/elec_mag: /opt/ros/noetic/lib/librostime.so
devel/lib/rs232_pkg/elec_mag: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
devel/lib/rs232_pkg/elec_mag: /opt/ros/noetic/lib/libcpp_common.so
devel/lib/rs232_pkg/elec_mag: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
devel/lib/rs232_pkg/elec_mag: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
devel/lib/rs232_pkg/elec_mag: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/rs232_pkg/elec_mag: rs232_pkg/CMakeFiles/elec_mag.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robot5/robot5_ws/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../devel/lib/rs232_pkg/elec_mag"
	cd /home/robot5/robot5_ws/src/build/rs232_pkg && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/elec_mag.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
rs232_pkg/CMakeFiles/elec_mag.dir/build: devel/lib/rs232_pkg/elec_mag

.PHONY : rs232_pkg/CMakeFiles/elec_mag.dir/build

rs232_pkg/CMakeFiles/elec_mag.dir/clean:
	cd /home/robot5/robot5_ws/src/build/rs232_pkg && $(CMAKE_COMMAND) -P CMakeFiles/elec_mag.dir/cmake_clean.cmake
.PHONY : rs232_pkg/CMakeFiles/elec_mag.dir/clean

rs232_pkg/CMakeFiles/elec_mag.dir/depend:
	cd /home/robot5/robot5_ws/src/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot5/robot5_ws/src /home/robot5/robot5_ws/src/rs232_pkg /home/robot5/robot5_ws/src/build /home/robot5/robot5_ws/src/build/rs232_pkg /home/robot5/robot5_ws/src/build/rs232_pkg/CMakeFiles/elec_mag.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : rs232_pkg/CMakeFiles/elec_mag.dir/depend

