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

# Utility rule file for x10_slider_masks.

# Include the progress variables for this target.
include radio/src/bitmaps/480x272/CMakeFiles/x10_slider_masks.dir/progress.make

radio/src/bitmaps/480x272/CMakeFiles/x10_slider_masks: radio/src/bitmaps/480x272/bar_left.lbm
radio/src/bitmaps/480x272/CMakeFiles/x10_slider_masks: radio/src/bitmaps/480x272/bar_right.lbm
radio/src/bitmaps/480x272/CMakeFiles/x10_slider_masks: radio/src/bitmaps/480x272/point_in.lbm
radio/src/bitmaps/480x272/CMakeFiles/x10_slider_masks: radio/src/bitmaps/480x272/point_mid.lbm
radio/src/bitmaps/480x272/CMakeFiles/x10_slider_masks: radio/src/bitmaps/480x272/point_out.lbm


radio/src/bitmaps/480x272/bar_left.lbm: ../radio/src/bitmaps/480x272/slider/bar_left.png
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating bar_left.lbm"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/480x272 && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/img2lbm.py /home/ros/Downloads/opentx/radio/src/bitmaps/480x272/slider/bar_left.png bar_left.lbm 480 8bits

radio/src/bitmaps/480x272/bar_right.lbm: ../radio/src/bitmaps/480x272/slider/bar_right.png
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating bar_right.lbm"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/480x272 && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/img2lbm.py /home/ros/Downloads/opentx/radio/src/bitmaps/480x272/slider/bar_right.png bar_right.lbm 480 8bits

radio/src/bitmaps/480x272/point_in.lbm: ../radio/src/bitmaps/480x272/slider/point_in.png
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating point_in.lbm"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/480x272 && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/img2lbm.py /home/ros/Downloads/opentx/radio/src/bitmaps/480x272/slider/point_in.png point_in.lbm 480 8bits

radio/src/bitmaps/480x272/point_mid.lbm: ../radio/src/bitmaps/480x272/slider/point_mid.png
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating point_mid.lbm"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/480x272 && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/img2lbm.py /home/ros/Downloads/opentx/radio/src/bitmaps/480x272/slider/point_mid.png point_mid.lbm 480 8bits

radio/src/bitmaps/480x272/point_out.lbm: ../radio/src/bitmaps/480x272/slider/point_out.png
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating point_out.lbm"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/480x272 && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/img2lbm.py /home/ros/Downloads/opentx/radio/src/bitmaps/480x272/slider/point_out.png point_out.lbm 480 8bits

x10_slider_masks: radio/src/bitmaps/480x272/CMakeFiles/x10_slider_masks
x10_slider_masks: radio/src/bitmaps/480x272/bar_left.lbm
x10_slider_masks: radio/src/bitmaps/480x272/bar_right.lbm
x10_slider_masks: radio/src/bitmaps/480x272/point_in.lbm
x10_slider_masks: radio/src/bitmaps/480x272/point_mid.lbm
x10_slider_masks: radio/src/bitmaps/480x272/point_out.lbm
x10_slider_masks: radio/src/bitmaps/480x272/CMakeFiles/x10_slider_masks.dir/build.make

.PHONY : x10_slider_masks

# Rule to build all files generated by this target.
radio/src/bitmaps/480x272/CMakeFiles/x10_slider_masks.dir/build: x10_slider_masks

.PHONY : radio/src/bitmaps/480x272/CMakeFiles/x10_slider_masks.dir/build

radio/src/bitmaps/480x272/CMakeFiles/x10_slider_masks.dir/clean:
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/480x272 && $(CMAKE_COMMAND) -P CMakeFiles/x10_slider_masks.dir/cmake_clean.cmake
.PHONY : radio/src/bitmaps/480x272/CMakeFiles/x10_slider_masks.dir/clean

radio/src/bitmaps/480x272/CMakeFiles/x10_slider_masks.dir/depend:
	cd /home/ros/Downloads/opentx/build-taranis-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/Downloads/opentx /home/ros/Downloads/opentx/radio/src/bitmaps/480x272 /home/ros/Downloads/opentx/build-taranis-debug /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/480x272 /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/480x272/CMakeFiles/x10_slider_masks.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : radio/src/bitmaps/480x272/CMakeFiles/x10_slider_masks.dir/depend

