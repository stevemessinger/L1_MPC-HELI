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

# Utility rule file for taranis_xbm.

# Include the progress variables for this target.
include radio/src/bitmaps/212x64/CMakeFiles/taranis_xbm.dir/progress.make

radio/src/bitmaps/212x64/CMakeFiles/taranis_xbm: radio/src/bitmaps/212x64/sticks.lbm


radio/src/bitmaps/212x64/sticks.lbm: ../radio/src/bitmaps/sticks.xbm
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating sticks.lbm"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/212x64 && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/img2lbm.py /home/ros/Downloads/opentx/radio/src/bitmaps/sticks.xbm sticks.lbm 128 1bit 4

taranis_xbm: radio/src/bitmaps/212x64/CMakeFiles/taranis_xbm
taranis_xbm: radio/src/bitmaps/212x64/sticks.lbm
taranis_xbm: radio/src/bitmaps/212x64/CMakeFiles/taranis_xbm.dir/build.make

.PHONY : taranis_xbm

# Rule to build all files generated by this target.
radio/src/bitmaps/212x64/CMakeFiles/taranis_xbm.dir/build: taranis_xbm

.PHONY : radio/src/bitmaps/212x64/CMakeFiles/taranis_xbm.dir/build

radio/src/bitmaps/212x64/CMakeFiles/taranis_xbm.dir/clean:
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/212x64 && $(CMAKE_COMMAND) -P CMakeFiles/taranis_xbm.dir/cmake_clean.cmake
.PHONY : radio/src/bitmaps/212x64/CMakeFiles/taranis_xbm.dir/clean

radio/src/bitmaps/212x64/CMakeFiles/taranis_xbm.dir/depend:
	cd /home/ros/Downloads/opentx/build-taranis-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/Downloads/opentx /home/ros/Downloads/opentx/radio/src/bitmaps/212x64 /home/ros/Downloads/opentx/build-taranis-debug /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/212x64 /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/212x64/CMakeFiles/taranis_xbm.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : radio/src/bitmaps/212x64/CMakeFiles/taranis_xbm.dir/depend

