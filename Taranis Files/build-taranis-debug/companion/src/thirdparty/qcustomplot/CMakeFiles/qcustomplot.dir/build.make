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
CMAKE_SOURCE_DIR = /home/ros/Downloads/opentx

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ros/Downloads/opentx/build-taranis-debug

# Include any dependencies generated for this target.
include companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/depend.make

# Include the progress variables for this target.
include companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/progress.make

# Include the compile flags for this target's objects.
include companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/flags.make

companion/src/thirdparty/qcustomplot/moc_qcustomplot.cpp: ../companion/src/thirdparty/qcustomplot/qcustomplot.h
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating moc_qcustomplot.cpp"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/qcustomplot && /usr/lib/qt5/bin/moc @/home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/qcustomplot/moc_qcustomplot.cpp_parameters

companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/qcustomplot.cpp.o: companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/flags.make
companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/qcustomplot.cpp.o: ../companion/src/thirdparty/qcustomplot/qcustomplot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/qcustomplot.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/qcustomplot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/qcustomplot.dir/qcustomplot.cpp.o -c /home/ros/Downloads/opentx/companion/src/thirdparty/qcustomplot/qcustomplot.cpp

companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/qcustomplot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/qcustomplot.dir/qcustomplot.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/qcustomplot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/companion/src/thirdparty/qcustomplot/qcustomplot.cpp > CMakeFiles/qcustomplot.dir/qcustomplot.cpp.i

companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/qcustomplot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/qcustomplot.dir/qcustomplot.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/qcustomplot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/companion/src/thirdparty/qcustomplot/qcustomplot.cpp -o CMakeFiles/qcustomplot.dir/qcustomplot.cpp.s

companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/moc_qcustomplot.cpp.o: companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/flags.make
companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/moc_qcustomplot.cpp.o: companion/src/thirdparty/qcustomplot/moc_qcustomplot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/moc_qcustomplot.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/qcustomplot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/qcustomplot.dir/moc_qcustomplot.cpp.o -c /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/qcustomplot/moc_qcustomplot.cpp

companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/moc_qcustomplot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/qcustomplot.dir/moc_qcustomplot.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/qcustomplot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/qcustomplot/moc_qcustomplot.cpp > CMakeFiles/qcustomplot.dir/moc_qcustomplot.cpp.i

companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/moc_qcustomplot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/qcustomplot.dir/moc_qcustomplot.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/qcustomplot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/qcustomplot/moc_qcustomplot.cpp -o CMakeFiles/qcustomplot.dir/moc_qcustomplot.cpp.s

# Object files for target qcustomplot
qcustomplot_OBJECTS = \
"CMakeFiles/qcustomplot.dir/qcustomplot.cpp.o" \
"CMakeFiles/qcustomplot.dir/moc_qcustomplot.cpp.o"

# External object files for target qcustomplot
qcustomplot_EXTERNAL_OBJECTS =

companion/src/thirdparty/qcustomplot/libqcustomplot.a: companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/qcustomplot.cpp.o
companion/src/thirdparty/qcustomplot/libqcustomplot.a: companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/moc_qcustomplot.cpp.o
companion/src/thirdparty/qcustomplot/libqcustomplot.a: companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/build.make
companion/src/thirdparty/qcustomplot/libqcustomplot.a: companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libqcustomplot.a"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/qcustomplot && $(CMAKE_COMMAND) -P CMakeFiles/qcustomplot.dir/cmake_clean_target.cmake
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/qcustomplot && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/qcustomplot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/build: companion/src/thirdparty/qcustomplot/libqcustomplot.a

.PHONY : companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/build

companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/clean:
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/qcustomplot && $(CMAKE_COMMAND) -P CMakeFiles/qcustomplot.dir/cmake_clean.cmake
.PHONY : companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/clean

companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/depend: companion/src/thirdparty/qcustomplot/moc_qcustomplot.cpp
	cd /home/ros/Downloads/opentx/build-taranis-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/Downloads/opentx /home/ros/Downloads/opentx/companion/src/thirdparty/qcustomplot /home/ros/Downloads/opentx/build-taranis-debug /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/qcustomplot /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : companion/src/thirdparty/qcustomplot/CMakeFiles/qcustomplot.dir/depend

