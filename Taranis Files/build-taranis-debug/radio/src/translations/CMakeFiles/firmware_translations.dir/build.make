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

# Utility rule file for firmware_translations.

# Include the progress variables for this target.
include radio/src/translations/CMakeFiles/firmware_translations.dir/progress.make

radio/src/translations/CMakeFiles/firmware_translations: radio/src/translations/cz.h
radio/src/translations/CMakeFiles/firmware_translations: radio/src/translations/de.h
radio/src/translations/CMakeFiles/firmware_translations: radio/src/translations/en.h
radio/src/translations/CMakeFiles/firmware_translations: radio/src/translations/es.h
radio/src/translations/CMakeFiles/firmware_translations: radio/src/translations/fi.h
radio/src/translations/CMakeFiles/firmware_translations: radio/src/translations/fr.h
radio/src/translations/CMakeFiles/firmware_translations: radio/src/translations/it.h
radio/src/translations/CMakeFiles/firmware_translations: radio/src/translations/nl.h
radio/src/translations/CMakeFiles/firmware_translations: radio/src/translations/pl.h
radio/src/translations/CMakeFiles/firmware_translations: radio/src/translations/pt.h
radio/src/translations/CMakeFiles/firmware_translations: radio/src/translations/se.h


radio/src/translations/cz.h: ../radio/src/translations/cz.h.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating cz.h"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/translations && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/translate.py /home/ros/Downloads/opentx/radio/src/translations/cz.h.txt cz.h cz_reduced

radio/src/translations/de.h: ../radio/src/translations/de.h.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating de.h"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/translations && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/translate.py /home/ros/Downloads/opentx/radio/src/translations/de.h.txt de.h de

radio/src/translations/en.h: ../radio/src/translations/en.h.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating en.h"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/translations && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/translate.py /home/ros/Downloads/opentx/radio/src/translations/en.h.txt en.h en

radio/src/translations/es.h: ../radio/src/translations/es.h.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating es.h"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/translations && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/translate.py /home/ros/Downloads/opentx/radio/src/translations/es.h.txt es.h es

radio/src/translations/fi.h: ../radio/src/translations/fi.h.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating fi.h"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/translations && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/translate.py /home/ros/Downloads/opentx/radio/src/translations/fi.h.txt fi.h fi

radio/src/translations/fr.h: ../radio/src/translations/fr.h.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Generating fr.h"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/translations && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/translate.py /home/ros/Downloads/opentx/radio/src/translations/fr.h.txt fr.h fr

radio/src/translations/it.h: ../radio/src/translations/it.h.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Generating it.h"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/translations && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/translate.py /home/ros/Downloads/opentx/radio/src/translations/it.h.txt it.h it

radio/src/translations/nl.h: ../radio/src/translations/nl.h.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Generating nl.h"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/translations && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/translate.py /home/ros/Downloads/opentx/radio/src/translations/nl.h.txt nl.h nl

radio/src/translations/pl.h: ../radio/src/translations/pl.h.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Generating pl.h"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/translations && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/translate.py /home/ros/Downloads/opentx/radio/src/translations/pl.h.txt pl.h pl

radio/src/translations/pt.h: ../radio/src/translations/pt.h.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Generating pt.h"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/translations && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/translate.py /home/ros/Downloads/opentx/radio/src/translations/pt.h.txt pt.h pt

radio/src/translations/se.h: ../radio/src/translations/se.h.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros/Downloads/opentx/build-taranis-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Generating se.h"
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/translations && /usr/bin/python3.8 /home/ros/Downloads/opentx/radio/util/translate.py /home/ros/Downloads/opentx/radio/src/translations/se.h.txt se.h se

firmware_translations: radio/src/translations/CMakeFiles/firmware_translations
firmware_translations: radio/src/translations/cz.h
firmware_translations: radio/src/translations/de.h
firmware_translations: radio/src/translations/en.h
firmware_translations: radio/src/translations/es.h
firmware_translations: radio/src/translations/fi.h
firmware_translations: radio/src/translations/fr.h
firmware_translations: radio/src/translations/it.h
firmware_translations: radio/src/translations/nl.h
firmware_translations: radio/src/translations/pl.h
firmware_translations: radio/src/translations/pt.h
firmware_translations: radio/src/translations/se.h
firmware_translations: radio/src/translations/CMakeFiles/firmware_translations.dir/build.make

.PHONY : firmware_translations

# Rule to build all files generated by this target.
radio/src/translations/CMakeFiles/firmware_translations.dir/build: firmware_translations

.PHONY : radio/src/translations/CMakeFiles/firmware_translations.dir/build

radio/src/translations/CMakeFiles/firmware_translations.dir/clean:
	cd /home/ros/Downloads/opentx/build-taranis-debug/radio/src/translations && $(CMAKE_COMMAND) -P CMakeFiles/firmware_translations.dir/cmake_clean.cmake
.PHONY : radio/src/translations/CMakeFiles/firmware_translations.dir/clean

radio/src/translations/CMakeFiles/firmware_translations.dir/depend:
	cd /home/ros/Downloads/opentx/build-taranis-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/Downloads/opentx /home/ros/Downloads/opentx/radio/src/translations /home/ros/Downloads/opentx/build-taranis-debug /home/ros/Downloads/opentx/build-taranis-debug/radio/src/translations /home/ros/Downloads/opentx/build-taranis-debug/radio/src/translations/CMakeFiles/firmware_translations.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : radio/src/translations/CMakeFiles/firmware_translations.dir/depend

