# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /home/lining/Documents/CLion-2018.3.4/clion-2018.3.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/lining/Documents/CLion-2018.3.4/clion-2018.3.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lining/Onboard-SDK-3.8

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lining/Onboard-SDK-3.8/cmake-build-debug

# Include any dependencies generated for this target.
include sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/depend.make

# Include the progress variables for this target.
include sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/progress.make

# Include the compile flags for this target's objects.
include sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/flags.make

sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_environment.cpp.o: sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/flags.make
sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_environment.cpp.o: ../sample/linux/common/dji_linux_environment.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lining/Onboard-SDK-3.8/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_environment.cpp.o"
	cd /home/lining/Onboard-SDK-3.8/cmake-build-debug/sample/linux/telemetry && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_environment.cpp.o -c /home/lining/Onboard-SDK-3.8/sample/linux/common/dji_linux_environment.cpp

sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_environment.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_environment.cpp.i"
	cd /home/lining/Onboard-SDK-3.8/cmake-build-debug/sample/linux/telemetry && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lining/Onboard-SDK-3.8/sample/linux/common/dji_linux_environment.cpp > CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_environment.cpp.i

sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_environment.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_environment.cpp.s"
	cd /home/lining/Onboard-SDK-3.8/cmake-build-debug/sample/linux/telemetry && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lining/Onboard-SDK-3.8/sample/linux/common/dji_linux_environment.cpp -o CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_environment.cpp.s

sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_helpers.cpp.o: sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/flags.make
sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_helpers.cpp.o: ../sample/linux/common/dji_linux_helpers.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lining/Onboard-SDK-3.8/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_helpers.cpp.o"
	cd /home/lining/Onboard-SDK-3.8/cmake-build-debug/sample/linux/telemetry && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_helpers.cpp.o -c /home/lining/Onboard-SDK-3.8/sample/linux/common/dji_linux_helpers.cpp

sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_helpers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_helpers.cpp.i"
	cd /home/lining/Onboard-SDK-3.8/cmake-build-debug/sample/linux/telemetry && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lining/Onboard-SDK-3.8/sample/linux/common/dji_linux_helpers.cpp > CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_helpers.cpp.i

sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_helpers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_helpers.cpp.s"
	cd /home/lining/Onboard-SDK-3.8/cmake-build-debug/sample/linux/telemetry && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lining/Onboard-SDK-3.8/sample/linux/common/dji_linux_helpers.cpp -o CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_helpers.cpp.s

sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/main.cpp.o: sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/flags.make
sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/main.cpp.o: ../sample/linux/telemetry/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lining/Onboard-SDK-3.8/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/main.cpp.o"
	cd /home/lining/Onboard-SDK-3.8/cmake-build-debug/sample/linux/telemetry && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/djiosdk-telemetry-sample.dir/main.cpp.o -c /home/lining/Onboard-SDK-3.8/sample/linux/telemetry/main.cpp

sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/djiosdk-telemetry-sample.dir/main.cpp.i"
	cd /home/lining/Onboard-SDK-3.8/cmake-build-debug/sample/linux/telemetry && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lining/Onboard-SDK-3.8/sample/linux/telemetry/main.cpp > CMakeFiles/djiosdk-telemetry-sample.dir/main.cpp.i

sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/djiosdk-telemetry-sample.dir/main.cpp.s"
	cd /home/lining/Onboard-SDK-3.8/cmake-build-debug/sample/linux/telemetry && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lining/Onboard-SDK-3.8/sample/linux/telemetry/main.cpp -o CMakeFiles/djiosdk-telemetry-sample.dir/main.cpp.s

sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/telemetry_sample.cpp.o: sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/flags.make
sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/telemetry_sample.cpp.o: ../sample/linux/telemetry/telemetry_sample.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lining/Onboard-SDK-3.8/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/telemetry_sample.cpp.o"
	cd /home/lining/Onboard-SDK-3.8/cmake-build-debug/sample/linux/telemetry && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/djiosdk-telemetry-sample.dir/telemetry_sample.cpp.o -c /home/lining/Onboard-SDK-3.8/sample/linux/telemetry/telemetry_sample.cpp

sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/telemetry_sample.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/djiosdk-telemetry-sample.dir/telemetry_sample.cpp.i"
	cd /home/lining/Onboard-SDK-3.8/cmake-build-debug/sample/linux/telemetry && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lining/Onboard-SDK-3.8/sample/linux/telemetry/telemetry_sample.cpp > CMakeFiles/djiosdk-telemetry-sample.dir/telemetry_sample.cpp.i

sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/telemetry_sample.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/djiosdk-telemetry-sample.dir/telemetry_sample.cpp.s"
	cd /home/lining/Onboard-SDK-3.8/cmake-build-debug/sample/linux/telemetry && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lining/Onboard-SDK-3.8/sample/linux/telemetry/telemetry_sample.cpp -o CMakeFiles/djiosdk-telemetry-sample.dir/telemetry_sample.cpp.s

# Object files for target djiosdk-telemetry-sample
djiosdk__telemetry__sample_OBJECTS = \
"CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_environment.cpp.o" \
"CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_helpers.cpp.o" \
"CMakeFiles/djiosdk-telemetry-sample.dir/main.cpp.o" \
"CMakeFiles/djiosdk-telemetry-sample.dir/telemetry_sample.cpp.o"

# External object files for target djiosdk-telemetry-sample
djiosdk__telemetry__sample_EXTERNAL_OBJECTS =

bin/djiosdk-telemetry-sample: sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_environment.cpp.o
bin/djiosdk-telemetry-sample: sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/__/common/dji_linux_helpers.cpp.o
bin/djiosdk-telemetry-sample: sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/main.cpp.o
bin/djiosdk-telemetry-sample: sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/telemetry_sample.cpp.o
bin/djiosdk-telemetry-sample: sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/build.make
bin/djiosdk-telemetry-sample: libs/libdjiosdk-core.a
bin/djiosdk-telemetry-sample: sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lining/Onboard-SDK-3.8/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable ../../../bin/djiosdk-telemetry-sample"
	cd /home/lining/Onboard-SDK-3.8/cmake-build-debug/sample/linux/telemetry && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/djiosdk-telemetry-sample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/build: bin/djiosdk-telemetry-sample

.PHONY : sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/build

sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/clean:
	cd /home/lining/Onboard-SDK-3.8/cmake-build-debug/sample/linux/telemetry && $(CMAKE_COMMAND) -P CMakeFiles/djiosdk-telemetry-sample.dir/cmake_clean.cmake
.PHONY : sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/clean

sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/depend:
	cd /home/lining/Onboard-SDK-3.8/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lining/Onboard-SDK-3.8 /home/lining/Onboard-SDK-3.8/sample/linux/telemetry /home/lining/Onboard-SDK-3.8/cmake-build-debug /home/lining/Onboard-SDK-3.8/cmake-build-debug/sample/linux/telemetry /home/lining/Onboard-SDK-3.8/cmake-build-debug/sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : sample/linux/telemetry/CMakeFiles/djiosdk-telemetry-sample.dir/depend
