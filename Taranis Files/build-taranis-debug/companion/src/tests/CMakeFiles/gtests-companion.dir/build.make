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
include companion/src/tests/CMakeFiles/gtests-companion.dir/depend.make

# Include the progress variables for this target.
include companion/src/tests/CMakeFiles/gtests-companion.dir/progress.make

# Include the compile flags for this target's objects.
include companion/src/tests/CMakeFiles/gtests-companion.dir/flags.make

companion/src/tests/CMakeFiles/gtests-companion.dir/conversions.cpp.o: companion/src/tests/CMakeFiles/gtests-companion.dir/flags.make
companion/src/tests/CMakeFiles/gtests-companion.dir/conversions.cpp.o: ../companion/src/tests/conversions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object companion/src/tests/CMakeFiles/gtests-companion.dir/conversions.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gtests-companion.dir/conversions.cpp.o -c /home/ros/Downloads/opentx/companion/src/tests/conversions.cpp

companion/src/tests/CMakeFiles/gtests-companion.dir/conversions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gtests-companion.dir/conversions.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/companion/src/tests/conversions.cpp > CMakeFiles/gtests-companion.dir/conversions.cpp.i

companion/src/tests/CMakeFiles/gtests-companion.dir/conversions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gtests-companion.dir/conversions.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/companion/src/tests/conversions.cpp -o CMakeFiles/gtests-companion.dir/conversions.cpp.s

companion/src/tests/CMakeFiles/gtests-companion.dir/gtests.cpp.o: companion/src/tests/CMakeFiles/gtests-companion.dir/flags.make
companion/src/tests/CMakeFiles/gtests-companion.dir/gtests.cpp.o: ../companion/src/tests/gtests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object companion/src/tests/CMakeFiles/gtests-companion.dir/gtests.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gtests-companion.dir/gtests.cpp.o -c /home/ros/Downloads/opentx/companion/src/tests/gtests.cpp

companion/src/tests/CMakeFiles/gtests-companion.dir/gtests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gtests-companion.dir/gtests.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/companion/src/tests/gtests.cpp > CMakeFiles/gtests-companion.dir/gtests.cpp.i

companion/src/tests/CMakeFiles/gtests-companion.dir/gtests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gtests-companion.dir/gtests.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/companion/src/tests/gtests.cpp -o CMakeFiles/gtests-companion.dir/gtests.cpp.s

# Object files for target gtests-companion
gtests__companion_OBJECTS = \
"CMakeFiles/gtests-companion.dir/conversions.cpp.o" \
"CMakeFiles/gtests-companion.dir/gtests.cpp.o"

# External object files for target gtests-companion
gtests__companion_EXTERNAL_OBJECTS =

gtests-companion: companion/src/tests/CMakeFiles/gtests-companion.dir/conversions.cpp.o
gtests-companion: companion/src/tests/CMakeFiles/gtests-companion.dir/gtests.cpp.o
gtests-companion: companion/src/tests/CMakeFiles/gtests-companion.dir/build.make
gtests-companion: companion/src/tests/libgtests-companion-lib.a
gtests-companion: companion/src/simulation/libsimulation.a
gtests-companion: companion/src/firmwares/libfirmwares.a
gtests-companion: companion/src/storage/libstorage.a
gtests-companion: companion/src/libcommon.a
gtests-companion: companion/src/datamodels/libdatamodels.a
gtests-companion: companion/src/simulation/libsimulation.a
gtests-companion: companion/src/firmwares/libfirmwares.a
gtests-companion: companion/src/storage/libstorage.a
gtests-companion: companion/src/libcommon.a
gtests-companion: companion/src/datamodels/libdatamodels.a
gtests-companion: /usr/lib/x86_64-linux-gnu/libQt5Svg.so.5.12.8
gtests-companion: /usr/lib/x86_64-linux-gnu/libQt5Xml.so.5.12.8
gtests-companion: companion/src/thirdparty/maxlibqt/src/widgets/libmaxLibQtWidgets.a
gtests-companion: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.12.8
gtests-companion: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.12.8
gtests-companion: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.12.8
gtests-companion: /usr/lib/x86_64-linux-gnu/libSDL.so
gtests-companion: companion/src/tests/CMakeFiles/gtests-companion.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../../../gtests-companion"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gtests-companion.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
companion/src/tests/CMakeFiles/gtests-companion.dir/build: gtests-companion

.PHONY : companion/src/tests/CMakeFiles/gtests-companion.dir/build

companion/src/tests/CMakeFiles/gtests-companion.dir/clean:
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/tests && $(CMAKE_COMMAND) -P CMakeFiles/gtests-companion.dir/cmake_clean.cmake
.PHONY : companion/src/tests/CMakeFiles/gtests-companion.dir/clean

companion/src/tests/CMakeFiles/gtests-companion.dir/depend:
	cd /home/ros/Downloads/opentx/build-taranis-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/Downloads/opentx /home/ros/Downloads/opentx/companion/src/tests /home/ros/Downloads/opentx/build-taranis-debug /home/ros/Downloads/opentx/build-taranis-debug/companion/src/tests /home/ros/Downloads/opentx/build-taranis-debug/companion/src/tests/CMakeFiles/gtests-companion.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : companion/src/tests/CMakeFiles/gtests-companion.dir/depend

