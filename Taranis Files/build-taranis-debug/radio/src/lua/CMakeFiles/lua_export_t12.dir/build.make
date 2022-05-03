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

# Utility rule file for lua_export_t12.

# Include the progress variables for this target.
include radio/src/lua/CMakeFiles/lua_export_t12.dir/progress.make

radio/src/lua/CMakeFiles/lua_export_t12: radio/src/lua/lua_exports_t12.inc


radio/src/lua/lua_exports_t12.inc:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating lua_exports_t12.inc"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/lua && arm-none-eabi-gcc -E -DCPUARM -DLUA -DLUA_INPUTS -DLUA_EXPORT_GENERATION -DSIMU -I/home/ros/Downloads/opentx/radio/src -I/home/ros/Downloads/opentx/radio/src/targets/taranis -I/home/ros/Downloads/opentx/radio/src/thirdparty -I/home/ros/Downloads/opentx/radio/src/targets/common/arm/stm32 -DPCBTARANIS -DPCBX7 -DRADIO_T12 -DEXPORT /home/ros/Downloads/opentx/radio/src/dataconstants.h > lua_exports_t12.txt
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/lua && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/luaexport.py 2.3.15 lua_exports_t12.txt lua_exports_t12.inc lua_fields_t12.txt

lua_export_t12: radio/src/lua/CMakeFiles/lua_export_t12
lua_export_t12: radio/src/lua/lua_exports_t12.inc
lua_export_t12: radio/src/lua/CMakeFiles/lua_export_t12.dir/build.make

.PHONY : lua_export_t12

# Rule to build all files generated by this target.
radio/src/lua/CMakeFiles/lua_export_t12.dir/build: lua_export_t12

.PHONY : radio/src/lua/CMakeFiles/lua_export_t12.dir/build

radio/src/lua/CMakeFiles/lua_export_t12.dir/clean:
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/lua && $(CMAKE_COMMAND) -P CMakeFiles/lua_export_t12.dir/cmake_clean.cmake
.PHONY : radio/src/lua/CMakeFiles/lua_export_t12.dir/clean

radio/src/lua/CMakeFiles/lua_export_t12.dir/depend:
	cd /home/ros/Downloads/opentx/build-taranis-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/Downloads/opentx /home/ros/Downloads/opentx/radio/src/lua /home/ros/Downloads/opentx/build-taranis-debug /home/ros/Downloads/opentx/build-taranis-debug/radio/src/lua /home/ros/Downloads/opentx/build-taranis-debug/radio/src/lua/CMakeFiles/lua_export_t12.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : radio/src/lua/CMakeFiles/lua_export_t12.dir/depend

