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
include companion/src/storage/CMakeFiles/storage.dir/depend.make

# Include the progress variables for this target.
include companion/src/storage/CMakeFiles/storage.dir/progress.make

# Include the compile flags for this target's objects.
include companion/src/storage/CMakeFiles/storage.dir/flags.make

companion/src/storage/moc_appdata.cpp: ../companion/src/storage/appdata.h
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating moc_appdata.cpp"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/lib/qt5/bin/moc @/home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage/moc_appdata.cpp_parameters

companion/src/storage/CMakeFiles/storage.dir/hexinterface.cpp.o: companion/src/storage/CMakeFiles/storage.dir/flags.make
companion/src/storage/CMakeFiles/storage.dir/hexinterface.cpp.o: ../companion/src/storage/hexinterface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object companion/src/storage/CMakeFiles/storage.dir/hexinterface.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/storage.dir/hexinterface.cpp.o -c /home/ros/Downloads/opentx/companion/src/storage/hexinterface.cpp

companion/src/storage/CMakeFiles/storage.dir/hexinterface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/storage.dir/hexinterface.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/companion/src/storage/hexinterface.cpp > CMakeFiles/storage.dir/hexinterface.cpp.i

companion/src/storage/CMakeFiles/storage.dir/hexinterface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/storage.dir/hexinterface.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/companion/src/storage/hexinterface.cpp -o CMakeFiles/storage.dir/hexinterface.cpp.s

companion/src/storage/CMakeFiles/storage.dir/mountlist.cpp.o: companion/src/storage/CMakeFiles/storage.dir/flags.make
companion/src/storage/CMakeFiles/storage.dir/mountlist.cpp.o: ../companion/src/storage/mountlist.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object companion/src/storage/CMakeFiles/storage.dir/mountlist.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/storage.dir/mountlist.cpp.o -c /home/ros/Downloads/opentx/companion/src/storage/mountlist.cpp

companion/src/storage/CMakeFiles/storage.dir/mountlist.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/storage.dir/mountlist.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/companion/src/storage/mountlist.cpp > CMakeFiles/storage.dir/mountlist.cpp.i

companion/src/storage/CMakeFiles/storage.dir/mountlist.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/storage.dir/mountlist.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/companion/src/storage/mountlist.cpp -o CMakeFiles/storage.dir/mountlist.cpp.s

companion/src/storage/CMakeFiles/storage.dir/rlefile.cpp.o: companion/src/storage/CMakeFiles/storage.dir/flags.make
companion/src/storage/CMakeFiles/storage.dir/rlefile.cpp.o: ../companion/src/storage/rlefile.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object companion/src/storage/CMakeFiles/storage.dir/rlefile.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/storage.dir/rlefile.cpp.o -c /home/ros/Downloads/opentx/companion/src/storage/rlefile.cpp

companion/src/storage/CMakeFiles/storage.dir/rlefile.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/storage.dir/rlefile.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/companion/src/storage/rlefile.cpp > CMakeFiles/storage.dir/rlefile.cpp.i

companion/src/storage/CMakeFiles/storage.dir/rlefile.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/storage.dir/rlefile.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/companion/src/storage/rlefile.cpp -o CMakeFiles/storage.dir/rlefile.cpp.s

companion/src/storage/CMakeFiles/storage.dir/appdata.cpp.o: companion/src/storage/CMakeFiles/storage.dir/flags.make
companion/src/storage/CMakeFiles/storage.dir/appdata.cpp.o: ../companion/src/storage/appdata.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object companion/src/storage/CMakeFiles/storage.dir/appdata.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/storage.dir/appdata.cpp.o -c /home/ros/Downloads/opentx/companion/src/storage/appdata.cpp

companion/src/storage/CMakeFiles/storage.dir/appdata.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/storage.dir/appdata.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/companion/src/storage/appdata.cpp > CMakeFiles/storage.dir/appdata.cpp.i

companion/src/storage/CMakeFiles/storage.dir/appdata.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/storage.dir/appdata.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/companion/src/storage/appdata.cpp -o CMakeFiles/storage.dir/appdata.cpp.s

companion/src/storage/CMakeFiles/storage.dir/firmwareinterface.cpp.o: companion/src/storage/CMakeFiles/storage.dir/flags.make
companion/src/storage/CMakeFiles/storage.dir/firmwareinterface.cpp.o: ../companion/src/storage/firmwareinterface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object companion/src/storage/CMakeFiles/storage.dir/firmwareinterface.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/storage.dir/firmwareinterface.cpp.o -c /home/ros/Downloads/opentx/companion/src/storage/firmwareinterface.cpp

companion/src/storage/CMakeFiles/storage.dir/firmwareinterface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/storage.dir/firmwareinterface.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/companion/src/storage/firmwareinterface.cpp > CMakeFiles/storage.dir/firmwareinterface.cpp.i

companion/src/storage/CMakeFiles/storage.dir/firmwareinterface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/storage.dir/firmwareinterface.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/companion/src/storage/firmwareinterface.cpp -o CMakeFiles/storage.dir/firmwareinterface.cpp.s

companion/src/storage/CMakeFiles/storage.dir/storage.cpp.o: companion/src/storage/CMakeFiles/storage.dir/flags.make
companion/src/storage/CMakeFiles/storage.dir/storage.cpp.o: ../companion/src/storage/storage.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object companion/src/storage/CMakeFiles/storage.dir/storage.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/storage.dir/storage.cpp.o -c /home/ros/Downloads/opentx/companion/src/storage/storage.cpp

companion/src/storage/CMakeFiles/storage.dir/storage.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/storage.dir/storage.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/companion/src/storage/storage.cpp > CMakeFiles/storage.dir/storage.cpp.i

companion/src/storage/CMakeFiles/storage.dir/storage.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/storage.dir/storage.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/companion/src/storage/storage.cpp -o CMakeFiles/storage.dir/storage.cpp.s

companion/src/storage/CMakeFiles/storage.dir/bineeprom.cpp.o: companion/src/storage/CMakeFiles/storage.dir/flags.make
companion/src/storage/CMakeFiles/storage.dir/bineeprom.cpp.o: ../companion/src/storage/bineeprom.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object companion/src/storage/CMakeFiles/storage.dir/bineeprom.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/storage.dir/bineeprom.cpp.o -c /home/ros/Downloads/opentx/companion/src/storage/bineeprom.cpp

companion/src/storage/CMakeFiles/storage.dir/bineeprom.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/storage.dir/bineeprom.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/companion/src/storage/bineeprom.cpp > CMakeFiles/storage.dir/bineeprom.cpp.i

companion/src/storage/CMakeFiles/storage.dir/bineeprom.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/storage.dir/bineeprom.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/companion/src/storage/bineeprom.cpp -o CMakeFiles/storage.dir/bineeprom.cpp.s

companion/src/storage/CMakeFiles/storage.dir/eepe.cpp.o: companion/src/storage/CMakeFiles/storage.dir/flags.make
companion/src/storage/CMakeFiles/storage.dir/eepe.cpp.o: ../companion/src/storage/eepe.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object companion/src/storage/CMakeFiles/storage.dir/eepe.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/storage.dir/eepe.cpp.o -c /home/ros/Downloads/opentx/companion/src/storage/eepe.cpp

companion/src/storage/CMakeFiles/storage.dir/eepe.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/storage.dir/eepe.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/companion/src/storage/eepe.cpp > CMakeFiles/storage.dir/eepe.cpp.i

companion/src/storage/CMakeFiles/storage.dir/eepe.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/storage.dir/eepe.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/companion/src/storage/eepe.cpp -o CMakeFiles/storage.dir/eepe.cpp.s

companion/src/storage/CMakeFiles/storage.dir/hexeeprom.cpp.o: companion/src/storage/CMakeFiles/storage.dir/flags.make
companion/src/storage/CMakeFiles/storage.dir/hexeeprom.cpp.o: ../companion/src/storage/hexeeprom.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object companion/src/storage/CMakeFiles/storage.dir/hexeeprom.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/storage.dir/hexeeprom.cpp.o -c /home/ros/Downloads/opentx/companion/src/storage/hexeeprom.cpp

companion/src/storage/CMakeFiles/storage.dir/hexeeprom.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/storage.dir/hexeeprom.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/companion/src/storage/hexeeprom.cpp > CMakeFiles/storage.dir/hexeeprom.cpp.i

companion/src/storage/CMakeFiles/storage.dir/hexeeprom.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/storage.dir/hexeeprom.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/companion/src/storage/hexeeprom.cpp -o CMakeFiles/storage.dir/hexeeprom.cpp.s

companion/src/storage/CMakeFiles/storage.dir/categorized.cpp.o: companion/src/storage/CMakeFiles/storage.dir/flags.make
companion/src/storage/CMakeFiles/storage.dir/categorized.cpp.o: ../companion/src/storage/categorized.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object companion/src/storage/CMakeFiles/storage.dir/categorized.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/storage.dir/categorized.cpp.o -c /home/ros/Downloads/opentx/companion/src/storage/categorized.cpp

companion/src/storage/CMakeFiles/storage.dir/categorized.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/storage.dir/categorized.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/companion/src/storage/categorized.cpp > CMakeFiles/storage.dir/categorized.cpp.i

companion/src/storage/CMakeFiles/storage.dir/categorized.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/storage.dir/categorized.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/companion/src/storage/categorized.cpp -o CMakeFiles/storage.dir/categorized.cpp.s

companion/src/storage/CMakeFiles/storage.dir/sdcard.cpp.o: companion/src/storage/CMakeFiles/storage.dir/flags.make
companion/src/storage/CMakeFiles/storage.dir/sdcard.cpp.o: ../companion/src/storage/sdcard.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object companion/src/storage/CMakeFiles/storage.dir/sdcard.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/storage.dir/sdcard.cpp.o -c /home/ros/Downloads/opentx/companion/src/storage/sdcard.cpp

companion/src/storage/CMakeFiles/storage.dir/sdcard.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/storage.dir/sdcard.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/companion/src/storage/sdcard.cpp > CMakeFiles/storage.dir/sdcard.cpp.i

companion/src/storage/CMakeFiles/storage.dir/sdcard.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/storage.dir/sdcard.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/companion/src/storage/sdcard.cpp -o CMakeFiles/storage.dir/sdcard.cpp.s

companion/src/storage/CMakeFiles/storage.dir/otx.cpp.o: companion/src/storage/CMakeFiles/storage.dir/flags.make
companion/src/storage/CMakeFiles/storage.dir/otx.cpp.o: ../companion/src/storage/otx.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object companion/src/storage/CMakeFiles/storage.dir/otx.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/storage.dir/otx.cpp.o -c /home/ros/Downloads/opentx/companion/src/storage/otx.cpp

companion/src/storage/CMakeFiles/storage.dir/otx.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/storage.dir/otx.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/companion/src/storage/otx.cpp > CMakeFiles/storage.dir/otx.cpp.i

companion/src/storage/CMakeFiles/storage.dir/otx.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/storage.dir/otx.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/companion/src/storage/otx.cpp -o CMakeFiles/storage.dir/otx.cpp.s

companion/src/storage/CMakeFiles/storage.dir/moc_appdata.cpp.o: companion/src/storage/CMakeFiles/storage.dir/flags.make
companion/src/storage/CMakeFiles/storage.dir/moc_appdata.cpp.o: companion/src/storage/moc_appdata.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object companion/src/storage/CMakeFiles/storage.dir/moc_appdata.cpp.o"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/storage.dir/moc_appdata.cpp.o -c /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage/moc_appdata.cpp

companion/src/storage/CMakeFiles/storage.dir/moc_appdata.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/storage.dir/moc_appdata.cpp.i"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage/moc_appdata.cpp > CMakeFiles/storage.dir/moc_appdata.cpp.i

companion/src/storage/CMakeFiles/storage.dir/moc_appdata.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/storage.dir/moc_appdata.cpp.s"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage/moc_appdata.cpp -o CMakeFiles/storage.dir/moc_appdata.cpp.s

# Object files for target storage
storage_OBJECTS = \
"CMakeFiles/storage.dir/hexinterface.cpp.o" \
"CMakeFiles/storage.dir/mountlist.cpp.o" \
"CMakeFiles/storage.dir/rlefile.cpp.o" \
"CMakeFiles/storage.dir/appdata.cpp.o" \
"CMakeFiles/storage.dir/firmwareinterface.cpp.o" \
"CMakeFiles/storage.dir/storage.cpp.o" \
"CMakeFiles/storage.dir/bineeprom.cpp.o" \
"CMakeFiles/storage.dir/eepe.cpp.o" \
"CMakeFiles/storage.dir/hexeeprom.cpp.o" \
"CMakeFiles/storage.dir/categorized.cpp.o" \
"CMakeFiles/storage.dir/sdcard.cpp.o" \
"CMakeFiles/storage.dir/otx.cpp.o" \
"CMakeFiles/storage.dir/moc_appdata.cpp.o"

# External object files for target storage
storage_EXTERNAL_OBJECTS =

companion/src/storage/libstorage.a: companion/src/storage/CMakeFiles/storage.dir/hexinterface.cpp.o
companion/src/storage/libstorage.a: companion/src/storage/CMakeFiles/storage.dir/mountlist.cpp.o
companion/src/storage/libstorage.a: companion/src/storage/CMakeFiles/storage.dir/rlefile.cpp.o
companion/src/storage/libstorage.a: companion/src/storage/CMakeFiles/storage.dir/appdata.cpp.o
companion/src/storage/libstorage.a: companion/src/storage/CMakeFiles/storage.dir/firmwareinterface.cpp.o
companion/src/storage/libstorage.a: companion/src/storage/CMakeFiles/storage.dir/storage.cpp.o
companion/src/storage/libstorage.a: companion/src/storage/CMakeFiles/storage.dir/bineeprom.cpp.o
companion/src/storage/libstorage.a: companion/src/storage/CMakeFiles/storage.dir/eepe.cpp.o
companion/src/storage/libstorage.a: companion/src/storage/CMakeFiles/storage.dir/hexeeprom.cpp.o
companion/src/storage/libstorage.a: companion/src/storage/CMakeFiles/storage.dir/categorized.cpp.o
companion/src/storage/libstorage.a: companion/src/storage/CMakeFiles/storage.dir/sdcard.cpp.o
companion/src/storage/libstorage.a: companion/src/storage/CMakeFiles/storage.dir/otx.cpp.o
companion/src/storage/libstorage.a: companion/src/storage/CMakeFiles/storage.dir/moc_appdata.cpp.o
companion/src/storage/libstorage.a: companion/src/storage/CMakeFiles/storage.dir/build.make
companion/src/storage/libstorage.a: companion/src/storage/CMakeFiles/storage.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Linking CXX static library libstorage.a"
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && $(CMAKE_COMMAND) -P CMakeFiles/storage.dir/cmake_clean_target.cmake
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/storage.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
companion/src/storage/CMakeFiles/storage.dir/build: companion/src/storage/libstorage.a

.PHONY : companion/src/storage/CMakeFiles/storage.dir/build

companion/src/storage/CMakeFiles/storage.dir/clean:
	cd /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage && $(CMAKE_COMMAND) -P CMakeFiles/storage.dir/cmake_clean.cmake
.PHONY : companion/src/storage/CMakeFiles/storage.dir/clean

companion/src/storage/CMakeFiles/storage.dir/depend: companion/src/storage/moc_appdata.cpp
	cd /home/ros/Downloads/opentx/build-taranis-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/Downloads/opentx /home/ros/Downloads/opentx/companion/src/storage /home/ros/Downloads/opentx/build-taranis-debug /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage /home/ros/Downloads/opentx/build-taranis-debug/companion/src/storage/CMakeFiles/storage.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : companion/src/storage/CMakeFiles/storage.dir/depend

