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

# Utility rule file for flash-jtag.

# Include the progress variables for this target.
include radio/src/CMakeFiles/flash-jtag.dir/progress.make

radio/src/CMakeFiles/flash-jtag: firmware.elf
	st-flash --reset write firmware.bin 0x8000000

flash-jtag: radio/src/CMakeFiles/flash-jtag
flash-jtag: radio/src/CMakeFiles/flash-jtag.dir/build.make

.PHONY : flash-jtag

# Rule to build all files generated by this target.
radio/src/CMakeFiles/flash-jtag.dir/build: flash-jtag

.PHONY : radio/src/CMakeFiles/flash-jtag.dir/build

radio/src/CMakeFiles/flash-jtag.dir/clean:
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src && $(CMAKE_COMMAND) -P CMakeFiles/flash-jtag.dir/cmake_clean.cmake
.PHONY : radio/src/CMakeFiles/flash-jtag.dir/clean

radio/src/CMakeFiles/flash-jtag.dir/depend:
	cd /home/ros/Downloads/opentx/build-taranis-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/Downloads/opentx /home/ros/Downloads/opentx/radio/src /home/ros/Downloads/opentx/build-taranis-debug /home/ros/Downloads/opentx/build-taranis-debug/radio/src /home/ros/Downloads/opentx/build-taranis-debug/radio/src/CMakeFiles/flash-jtag.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : radio/src/CMakeFiles/flash-jtag.dir/depend

