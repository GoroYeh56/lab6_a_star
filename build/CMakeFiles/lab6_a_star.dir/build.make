# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/goro/catkin_ws/src/lab6_a_star

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/goro/catkin_ws/src/lab6_a_star/build

# Include any dependencies generated for this target.
include CMakeFiles/lab6_a_star.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/lab6_a_star.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lab6_a_star.dir/flags.make

CMakeFiles/lab6_a_star.dir/src/lab6_a_star.cpp.o: CMakeFiles/lab6_a_star.dir/flags.make
CMakeFiles/lab6_a_star.dir/src/lab6_a_star.cpp.o: ../src/lab6_a_star.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/goro/catkin_ws/src/lab6_a_star/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/lab6_a_star.dir/src/lab6_a_star.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lab6_a_star.dir/src/lab6_a_star.cpp.o -c /home/goro/catkin_ws/src/lab6_a_star/src/lab6_a_star.cpp

CMakeFiles/lab6_a_star.dir/src/lab6_a_star.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lab6_a_star.dir/src/lab6_a_star.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/goro/catkin_ws/src/lab6_a_star/src/lab6_a_star.cpp > CMakeFiles/lab6_a_star.dir/src/lab6_a_star.cpp.i

CMakeFiles/lab6_a_star.dir/src/lab6_a_star.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lab6_a_star.dir/src/lab6_a_star.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/goro/catkin_ws/src/lab6_a_star/src/lab6_a_star.cpp -o CMakeFiles/lab6_a_star.dir/src/lab6_a_star.cpp.s

# Object files for target lab6_a_star
lab6_a_star_OBJECTS = \
"CMakeFiles/lab6_a_star.dir/src/lab6_a_star.cpp.o"

# External object files for target lab6_a_star
lab6_a_star_EXTERNAL_OBJECTS =

devel/lib/lab6_a_star/lab6_a_star: CMakeFiles/lab6_a_star.dir/src/lab6_a_star.cpp.o
devel/lib/lab6_a_star/lab6_a_star: CMakeFiles/lab6_a_star.dir/build.make
devel/lib/lab6_a_star/lab6_a_star: /opt/ros/melodic/lib/libroscpp.so
devel/lib/lab6_a_star/lab6_a_star: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/lab6_a_star/lab6_a_star: /opt/ros/melodic/lib/librosconsole.so
devel/lib/lab6_a_star/lab6_a_star: /opt/ros/melodic/lib/librosconsole_log4cxx.so
devel/lib/lab6_a_star/lab6_a_star: /opt/ros/melodic/lib/librosconsole_backend_interface.so
devel/lib/lab6_a_star/lab6_a_star: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/lab6_a_star/lab6_a_star: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/lab6_a_star/lab6_a_star: /opt/ros/melodic/lib/libxmlrpcpp.so
devel/lib/lab6_a_star/lab6_a_star: /opt/ros/melodic/lib/libroscpp_serialization.so
devel/lib/lab6_a_star/lab6_a_star: /opt/ros/melodic/lib/librostime.so
devel/lib/lab6_a_star/lab6_a_star: /opt/ros/melodic/lib/libcpp_common.so
devel/lib/lab6_a_star/lab6_a_star: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/lab6_a_star/lab6_a_star: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/lab6_a_star/lab6_a_star: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/lab6_a_star/lab6_a_star: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/lab6_a_star/lab6_a_star: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/lab6_a_star/lab6_a_star: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/lab6_a_star/lab6_a_star: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/lab6_a_star/lab6_a_star: CMakeFiles/lab6_a_star.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/goro/catkin_ws/src/lab6_a_star/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable devel/lib/lab6_a_star/lab6_a_star"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lab6_a_star.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lab6_a_star.dir/build: devel/lib/lab6_a_star/lab6_a_star

.PHONY : CMakeFiles/lab6_a_star.dir/build

CMakeFiles/lab6_a_star.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lab6_a_star.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lab6_a_star.dir/clean

CMakeFiles/lab6_a_star.dir/depend:
	cd /home/goro/catkin_ws/src/lab6_a_star/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/goro/catkin_ws/src/lab6_a_star /home/goro/catkin_ws/src/lab6_a_star /home/goro/catkin_ws/src/lab6_a_star/build /home/goro/catkin_ws/src/lab6_a_star/build /home/goro/catkin_ws/src/lab6_a_star/build/CMakeFiles/lab6_a_star.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lab6_a_star.dir/depend

