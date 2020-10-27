# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/nvidia/work/DJI_Flight_Ctrl

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nvidia/work/DJI_Flight_Ctrl/build

# Include any dependencies generated for this target.
include flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/depend.make

# Include the progress variables for this target.
include flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/progress.make

# Include the compile flags for this target's objects.
include flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flags.make

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.o: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flags.make
flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.o: ../flight-control/flight_control_interface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/work/DJI_Flight_Ctrl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.o"
	cd /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.o -c /home/nvidia/work/DJI_Flight_Ctrl/flight-control/flight_control_interface.cpp

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.i"
	cd /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/work/DJI_Flight_Ctrl/flight-control/flight_control_interface.cpp > CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.i

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.s"
	cd /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/work/DJI_Flight_Ctrl/flight-control/flight_control_interface.cpp -o CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.s

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.o.requires:

.PHONY : flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.o.requires

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.o.provides: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.o.requires
	$(MAKE) -f flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/build.make flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.o.provides.build
.PHONY : flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.o.provides

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.o.provides.build: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.o


flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.o: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flags.make
flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.o: ../flight-control/flight_control_sample.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/work/DJI_Flight_Ctrl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.o"
	cd /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.o -c /home/nvidia/work/DJI_Flight_Ctrl/flight-control/flight_control_sample.cpp

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.i"
	cd /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/work/DJI_Flight_Ctrl/flight-control/flight_control_sample.cpp > CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.i

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.s"
	cd /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/work/DJI_Flight_Ctrl/flight-control/flight_control_sample.cpp -o CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.s

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.o.requires:

.PHONY : flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.o.requires

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.o.provides: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.o.requires
	$(MAKE) -f flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/build.make flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.o.provides.build
.PHONY : flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.o.provides

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.o.provides.build: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.o


flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.o: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flags.make
flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.o: ../flight-control/main_ori.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/work/DJI_Flight_Ctrl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.o"
	cd /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.o -c /home/nvidia/work/DJI_Flight_Ctrl/flight-control/main_ori.cpp

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.i"
	cd /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/work/DJI_Flight_Ctrl/flight-control/main_ori.cpp > CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.i

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.s"
	cd /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/work/DJI_Flight_Ctrl/flight-control/main_ori.cpp -o CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.s

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.o.requires:

.PHONY : flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.o.requires

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.o.provides: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.o.requires
	$(MAKE) -f flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/build.make flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.o.provides.build
.PHONY : flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.o.provides

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.o.provides.build: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.o


flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.o: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flags.make
flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.o: ../common/dji_linux_environment.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/work/DJI_Flight_Ctrl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.o"
	cd /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.o -c /home/nvidia/work/DJI_Flight_Ctrl/common/dji_linux_environment.cpp

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.i"
	cd /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/work/DJI_Flight_Ctrl/common/dji_linux_environment.cpp > CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.i

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.s"
	cd /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/work/DJI_Flight_Ctrl/common/dji_linux_environment.cpp -o CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.s

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.o.requires:

.PHONY : flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.o.requires

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.o.provides: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.o.requires
	$(MAKE) -f flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/build.make flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.o.provides.build
.PHONY : flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.o.provides

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.o.provides.build: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.o


flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.o: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flags.make
flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.o: ../common/dji_linux_helpers.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/work/DJI_Flight_Ctrl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.o"
	cd /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.o -c /home/nvidia/work/DJI_Flight_Ctrl/common/dji_linux_helpers.cpp

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.i"
	cd /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/work/DJI_Flight_Ctrl/common/dji_linux_helpers.cpp > CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.i

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.s"
	cd /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/work/DJI_Flight_Ctrl/common/dji_linux_helpers.cpp -o CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.s

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.o.requires:

.PHONY : flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.o.requires

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.o.provides: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.o.requires
	$(MAKE) -f flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/build.make flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.o.provides.build
.PHONY : flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.o.provides

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.o.provides.build: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.o


# Object files for target djiosdk-flightcontrol-sample
djiosdk__flightcontrol__sample_OBJECTS = \
"CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.o" \
"CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.o" \
"CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.o" \
"CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.o" \
"CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.o"

# External object files for target djiosdk-flightcontrol-sample
djiosdk__flightcontrol__sample_EXTERNAL_OBJECTS =

bin/djiosdk-flightcontrol-sample: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.o
bin/djiosdk-flightcontrol-sample: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.o
bin/djiosdk-flightcontrol-sample: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.o
bin/djiosdk-flightcontrol-sample: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.o
bin/djiosdk-flightcontrol-sample: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.o
bin/djiosdk-flightcontrol-sample: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/build.make
bin/djiosdk-flightcontrol-sample: libs/libdjiosdk-core.a
bin/djiosdk-flightcontrol-sample: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nvidia/work/DJI_Flight_Ctrl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable ../bin/djiosdk-flightcontrol-sample"
	cd /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/djiosdk-flightcontrol-sample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/build: bin/djiosdk-flightcontrol-sample

.PHONY : flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/build

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/requires: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_interface.cpp.o.requires
flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/requires: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/flight_control_sample.cpp.o.requires
flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/requires: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/main_ori.cpp.o.requires
flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/requires: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_environment.cpp.o.requires
flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/requires: flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/__/common/dji_linux_helpers.cpp.o.requires

.PHONY : flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/requires

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/clean:
	cd /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control && $(CMAKE_COMMAND) -P CMakeFiles/djiosdk-flightcontrol-sample.dir/cmake_clean.cmake
.PHONY : flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/clean

flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/depend:
	cd /home/nvidia/work/DJI_Flight_Ctrl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nvidia/work/DJI_Flight_Ctrl /home/nvidia/work/DJI_Flight_Ctrl/flight-control /home/nvidia/work/DJI_Flight_Ctrl/build /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control /home/nvidia/work/DJI_Flight_Ctrl/build/flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : flight-control/CMakeFiles/djiosdk-flightcontrol-sample.dir/depend
