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
include companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/depend.make

# Include the progress variables for this target.
include companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/progress.make

# Include the compile flags for this target's objects.
include companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/flags.make

companion/src/thirdparty/maxlibqt/src/widgets/moc_ExportableTableView.cpp: ../companion/src/thirdparty/maxlibqt/src/widgets/ExportableTableView.h
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating moc_ExportableTableView.cpp"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets && /usr/lib/qt5/bin/moc @/home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets/moc_ExportableTableView.cpp_parameters

companion/src/thirdparty/maxlibqt/src/widgets/moc_TimerEdit.cpp: ../companion/src/thirdparty/maxlibqt/src/widgets/TimerEdit.h
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating moc_TimerEdit.cpp"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets && /usr/lib/qt5/bin/moc @/home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets/moc_TimerEdit.cpp_parameters

companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/ExportableTableView.cpp.o: companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/flags.make
companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/ExportableTableView.cpp.o: ../companion/src/thirdparty/maxlibqt/src/widgets/ExportableTableView.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/ExportableTableView.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/maxLibQtWidgets.dir/ExportableTableView.cpp.o -c /home/ros/Downloads/opentx/companion/src/thirdparty/maxlibqt/src/widgets/ExportableTableView.cpp

companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/ExportableTableView.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/maxLibQtWidgets.dir/ExportableTableView.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/companion/src/thirdparty/maxlibqt/src/widgets/ExportableTableView.cpp > CMakeFiles/maxLibQtWidgets.dir/ExportableTableView.cpp.i

companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/ExportableTableView.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/maxLibQtWidgets.dir/ExportableTableView.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/companion/src/thirdparty/maxlibqt/src/widgets/ExportableTableView.cpp -o CMakeFiles/maxLibQtWidgets.dir/ExportableTableView.cpp.s

companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/TimerEdit.cpp.o: companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/flags.make
companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/TimerEdit.cpp.o: ../companion/src/thirdparty/maxlibqt/src/widgets/TimerEdit.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/TimerEdit.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/maxLibQtWidgets.dir/TimerEdit.cpp.o -c /home/ros/Downloads/opentx/companion/src/thirdparty/maxlibqt/src/widgets/TimerEdit.cpp

companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/TimerEdit.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/maxLibQtWidgets.dir/TimerEdit.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/companion/src/thirdparty/maxlibqt/src/widgets/TimerEdit.cpp > CMakeFiles/maxLibQtWidgets.dir/TimerEdit.cpp.i

companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/TimerEdit.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/maxLibQtWidgets.dir/TimerEdit.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/companion/src/thirdparty/maxlibqt/src/widgets/TimerEdit.cpp -o CMakeFiles/maxLibQtWidgets.dir/TimerEdit.cpp.s

companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/moc_ExportableTableView.cpp.o: companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/flags.make
companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/moc_ExportableTableView.cpp.o: companion/src/thirdparty/maxlibqt/src/widgets/moc_ExportableTableView.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/moc_ExportableTableView.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/maxLibQtWidgets.dir/moc_ExportableTableView.cpp.o -c /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets/moc_ExportableTableView.cpp

companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/moc_ExportableTableView.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/maxLibQtWidgets.dir/moc_ExportableTableView.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets/moc_ExportableTableView.cpp > CMakeFiles/maxLibQtWidgets.dir/moc_ExportableTableView.cpp.i

companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/moc_ExportableTableView.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/maxLibQtWidgets.dir/moc_ExportableTableView.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets/moc_ExportableTableView.cpp -o CMakeFiles/maxLibQtWidgets.dir/moc_ExportableTableView.cpp.s

companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/moc_TimerEdit.cpp.o: companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/flags.make
companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/moc_TimerEdit.cpp.o: companion/src/thirdparty/maxlibqt/src/widgets/moc_TimerEdit.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/moc_TimerEdit.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/maxLibQtWidgets.dir/moc_TimerEdit.cpp.o -c /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets/moc_TimerEdit.cpp

companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/moc_TimerEdit.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/maxLibQtWidgets.dir/moc_TimerEdit.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets/moc_TimerEdit.cpp > CMakeFiles/maxLibQtWidgets.dir/moc_TimerEdit.cpp.i

companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/moc_TimerEdit.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/maxLibQtWidgets.dir/moc_TimerEdit.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets/moc_TimerEdit.cpp -o CMakeFiles/maxLibQtWidgets.dir/moc_TimerEdit.cpp.s

# Object files for target maxLibQtWidgets
maxLibQtWidgets_OBJECTS = \
"CMakeFiles/maxLibQtWidgets.dir/ExportableTableView.cpp.o" \
"CMakeFiles/maxLibQtWidgets.dir/TimerEdit.cpp.o" \
"CMakeFiles/maxLibQtWidgets.dir/moc_ExportableTableView.cpp.o" \
"CMakeFiles/maxLibQtWidgets.dir/moc_TimerEdit.cpp.o"

# External object files for target maxLibQtWidgets
maxLibQtWidgets_EXTERNAL_OBJECTS =

companion/src/thirdparty/maxlibqt/src/widgets/libmaxLibQtWidgets.a: companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/ExportableTableView.cpp.o
companion/src/thirdparty/maxlibqt/src/widgets/libmaxLibQtWidgets.a: companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/TimerEdit.cpp.o
companion/src/thirdparty/maxlibqt/src/widgets/libmaxLibQtWidgets.a: companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/moc_ExportableTableView.cpp.o
companion/src/thirdparty/maxlibqt/src/widgets/libmaxLibQtWidgets.a: companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/moc_TimerEdit.cpp.o
companion/src/thirdparty/maxlibqt/src/widgets/libmaxLibQtWidgets.a: companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/build.make
companion/src/thirdparty/maxlibqt/src/widgets/libmaxLibQtWidgets.a: companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX static library libmaxLibQtWidgets.a"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets && $(CMAKE_COMMAND) -P CMakeFiles/maxLibQtWidgets.dir/cmake_clean_target.cmake
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/maxLibQtWidgets.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/build: companion/src/thirdparty/maxlibqt/src/widgets/libmaxLibQtWidgets.a

.PHONY : companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/build

companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/clean:
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets && $(CMAKE_COMMAND) -P CMakeFiles/maxLibQtWidgets.dir/cmake_clean.cmake
.PHONY : companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/clean

companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/depend: companion/src/thirdparty/maxlibqt/src/widgets/moc_ExportableTableView.cpp
companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/depend: companion/src/thirdparty/maxlibqt/src/widgets/moc_TimerEdit.cpp
	cd /home/ros/Downloads/opentx/build-taranis-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/Downloads/opentx /home/ros/Downloads/opentx/companion/src/thirdparty/maxlibqt/src/widgets /home/ros/Downloads/opentx/build-taranis-debug /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets /home/ros/Downloads/opentx/build-taranis-debug/companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : companion/src/thirdparty/maxlibqt/src/widgets/CMakeFiles/maxLibQtWidgets.dir/depend

