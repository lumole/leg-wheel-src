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
include interface_pkg/CMakeFiles/interface.dir/depend.make

# Include the progress variables for this target.
include interface_pkg/CMakeFiles/interface.dir/progress.make

# Include the compile flags for this target's objects.
include interface_pkg/CMakeFiles/interface.dir/flags.make

interface_pkg/CMakeFiles/interface.dir/src/interface.cpp.o: interface_pkg/CMakeFiles/interface.dir/flags.make
interface_pkg/CMakeFiles/interface.dir/src/interface.cpp.o: ../interface_pkg/src/interface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot5/robot5_ws/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object interface_pkg/CMakeFiles/interface.dir/src/interface.cpp.o"
	cd /home/robot5/robot5_ws/src/build/interface_pkg && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/interface.dir/src/interface.cpp.o -c /home/robot5/robot5_ws/src/interface_pkg/src/interface.cpp

interface_pkg/CMakeFiles/interface.dir/src/interface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/interface.dir/src/interface.cpp.i"
	cd /home/robot5/robot5_ws/src/build/interface_pkg && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robot5/robot5_ws/src/interface_pkg/src/interface.cpp > CMakeFiles/interface.dir/src/interface.cpp.i

interface_pkg/CMakeFiles/interface.dir/src/interface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/interface.dir/src/interface.cpp.s"
	cd /home/robot5/robot5_ws/src/build/interface_pkg && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robot5/robot5_ws/src/interface_pkg/src/interface.cpp -o CMakeFiles/interface.dir/src/interface.cpp.s

# Object files for target interface
interface_OBJECTS = \
"CMakeFiles/interface.dir/src/interface.cpp.o"

# External object files for target interface
interface_EXTERNAL_OBJECTS =

devel/lib/libinterface.so: interface_pkg/CMakeFiles/interface.dir/src/interface.cpp.o
devel/lib/libinterface.so: interface_pkg/CMakeFiles/interface.dir/build.make
devel/lib/libinterface.so: interface_pkg/CMakeFiles/interface.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robot5/robot5_ws/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library ../devel/lib/libinterface.so"
	cd /home/robot5/robot5_ws/src/build/interface_pkg && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/interface.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
interface_pkg/CMakeFiles/interface.dir/build: devel/lib/libinterface.so

.PHONY : interface_pkg/CMakeFiles/interface.dir/build

interface_pkg/CMakeFiles/interface.dir/clean:
	cd /home/robot5/robot5_ws/src/build/interface_pkg && $(CMAKE_COMMAND) -P CMakeFiles/interface.dir/cmake_clean.cmake
.PHONY : interface_pkg/CMakeFiles/interface.dir/clean

interface_pkg/CMakeFiles/interface.dir/depend:
	cd /home/robot5/robot5_ws/src/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot5/robot5_ws/src /home/robot5/robot5_ws/src/interface_pkg /home/robot5/robot5_ws/src/build /home/robot5/robot5_ws/src/build/interface_pkg /home/robot5/robot5_ws/src/build/interface_pkg/CMakeFiles/interface.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : interface_pkg/CMakeFiles/interface.dir/depend

