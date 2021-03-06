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

# Utility rule file for x10_bootloader_icons.

# Include the progress variables for this target.
include radio/src/bitmaps/480x272/CMakeFiles/x10_bootloader_icons.dir/progress.make

radio/src/bitmaps/480x272/CMakeFiles/x10_bootloader_icons: radio/src/bitmaps/480x272/icon_error.lbm
radio/src/bitmaps/480x272/CMakeFiles/x10_bootloader_icons: radio/src/bitmaps/480x272/icon_exit.lbm
radio/src/bitmaps/480x272/CMakeFiles/x10_bootloader_icons: radio/src/bitmaps/480x272/icon_file.lbm
radio/src/bitmaps/480x272/CMakeFiles/x10_bootloader_icons: radio/src/bitmaps/480x272/icon_flash.lbm
radio/src/bitmaps/480x272/CMakeFiles/x10_bootloader_icons: radio/src/bitmaps/480x272/icon_ok.lbm
radio/src/bitmaps/480x272/CMakeFiles/x10_bootloader_icons: radio/src/bitmaps/480x272/icon_sd.lbm


radio/src/bitmaps/480x272/icon_error.lbm: ../radio/src/bitmaps/480x272/bootloader/icon_error.png
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating icon_error.lbm"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/480x272 && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/img2lbm.py /home/ros/Downloads/opentx/radio/src/bitmaps/480x272/bootloader/icon_error.png icon_error.lbm 480 8bits

radio/src/bitmaps/480x272/icon_exit.lbm: ../radio/src/bitmaps/480x272/bootloader/icon_exit.png
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating icon_exit.lbm"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/480x272 && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/img2lbm.py /home/ros/Downloads/opentx/radio/src/bitmaps/480x272/bootloader/icon_exit.png icon_exit.lbm 480 8bits

radio/src/bitmaps/480x272/icon_file.lbm: ../radio/src/bitmaps/480x272/bootloader/icon_file.png
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating icon_file.lbm"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/480x272 && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/img2lbm.py /home/ros/Downloads/opentx/radio/src/bitmaps/480x272/bootloader/icon_file.png icon_file.lbm 480 8bits

radio/src/bitmaps/480x272/icon_flash.lbm: ../radio/src/bitmaps/480x272/bootloader/icon_flash.png
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating icon_flash.lbm"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/480x272 && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/img2lbm.py /home/ros/Downloads/opentx/radio/src/bitmaps/480x272/bootloader/icon_flash.png icon_flash.lbm 480 8bits

radio/src/bitmaps/480x272/icon_ok.lbm: ../radio/src/bitmaps/480x272/bootloader/icon_ok.png
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating icon_ok.lbm"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/480x272 && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/img2lbm.py /home/ros/Downloads/opentx/radio/src/bitmaps/480x272/bootloader/icon_ok.png icon_ok.lbm 480 8bits

radio/src/bitmaps/480x272/icon_sd.lbm: ../radio/src/bitmaps/480x272/bootloader/icon_sd.png
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Generating icon_sd.lbm"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/480x272 && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/img2lbm.py /home/ros/Downloads/opentx/radio/src/bitmaps/480x272/bootloader/icon_sd.png icon_sd.lbm 480 8bits

x10_bootloader_icons: radio/src/bitmaps/480x272/CMakeFiles/x10_bootloader_icons
x10_bootloader_icons: radio/src/bitmaps/480x272/icon_error.lbm
x10_bootloader_icons: radio/src/bitmaps/480x272/icon_exit.lbm
x10_bootloader_icons: radio/src/bitmaps/480x272/icon_file.lbm
x10_bootloader_icons: radio/src/bitmaps/480x272/icon_flash.lbm
x10_bootloader_icons: radio/src/bitmaps/480x272/icon_ok.lbm
x10_bootloader_icons: radio/src/bitmaps/480x272/icon_sd.lbm
x10_bootloader_icons: radio/src/bitmaps/480x272/CMakeFiles/x10_bootloader_icons.dir/build.make

.PHONY : x10_bootloader_icons

# Rule to build all files generated by this target.
radio/src/bitmaps/480x272/CMakeFiles/x10_bootloader_icons.dir/build: x10_bootloader_icons

.PHONY : radio/src/bitmaps/480x272/CMakeFiles/x10_bootloader_icons.dir/build

radio/src/bitmaps/480x272/CMakeFiles/x10_bootloader_icons.dir/clean:
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/480x272 && $(CMAKE_COMMAND) -P CMakeFiles/x10_bootloader_icons.dir/cmake_clean.cmake
.PHONY : radio/src/bitmaps/480x272/CMakeFiles/x10_bootloader_icons.dir/clean

radio/src/bitmaps/480x272/CMakeFiles/x10_bootloader_icons.dir/depend:
	cd /home/ros/Downloads/opentx/build-taranis-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/Downloads/opentx /home/ros/Downloads/opentx/radio/src/bitmaps/480x272 /home/ros/Downloads/opentx/build-taranis-debug /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/480x272 /home/ros/Downloads/opentx/build-taranis-debug/radio/src/bitmaps/480x272/CMakeFiles/x10_bootloader_icons.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : radio/src/bitmaps/480x272/CMakeFiles/x10_bootloader_icons.dir/depend

