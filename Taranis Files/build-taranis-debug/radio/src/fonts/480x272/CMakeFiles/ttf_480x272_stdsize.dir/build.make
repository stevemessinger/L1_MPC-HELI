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

# Utility rule file for ttf_480x272_stdsize.

# Include the progress variables for this target.
include radio/src/fonts/480x272/CMakeFiles/ttf_480x272_stdsize.dir/progress.make

radio/src/fonts/480x272/CMakeFiles/ttf_480x272_stdsize:
	cd /home/ros/Downloads/opentx/radio/src && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/font2png.py Roboto/Roboto-Regular 16 1 /home/ros/Downloads/opentx/radio/src/fonts/480x272/font_stdsize

ttf_480x272_stdsize: radio/src/fonts/480x272/CMakeFiles/ttf_480x272_stdsize
ttf_480x272_stdsize: radio/src/fonts/480x272/CMakeFiles/ttf_480x272_stdsize.dir/build.make

.PHONY : ttf_480x272_stdsize

# Rule to build all files generated by this target.
radio/src/fonts/480x272/CMakeFiles/ttf_480x272_stdsize.dir/build: ttf_480x272_stdsize

.PHONY : radio/src/fonts/480x272/CMakeFiles/ttf_480x272_stdsize.dir/build

radio/src/fonts/480x272/CMakeFiles/ttf_480x272_stdsize.dir/clean:
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/fonts/480x272 && $(CMAKE_COMMAND) -P CMakeFiles/ttf_480x272_stdsize.dir/cmake_clean.cmake
.PHONY : radio/src/fonts/480x272/CMakeFiles/ttf_480x272_stdsize.dir/clean

radio/src/fonts/480x272/CMakeFiles/ttf_480x272_stdsize.dir/depend:
	cd /home/ros/Downloads/opentx/build-taranis-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/Downloads/opentx /home/ros/Downloads/opentx/radio/src/fonts/480x272 /home/ros/Downloads/opentx/build-taranis-debug /home/ros/Downloads/opentx/build-taranis-debug/radio/src/fonts/480x272 /home/ros/Downloads/opentx/build-taranis-debug/radio/src/fonts/480x272/CMakeFiles/ttf_480x272_stdsize.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : radio/src/fonts/480x272/CMakeFiles/ttf_480x272_stdsize.dir/depend

