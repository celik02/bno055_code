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
CMAKE_SOURCE_DIR = /home/ikon/rt_imu_data_cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ikon/rt_imu_data_cpp/build

# Include any dependencies generated for this target.
include CMakeFiles/READ_IMU.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/READ_IMU.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/READ_IMU.dir/flags.make

CMakeFiles/READ_IMU.dir/main.cpp.o: CMakeFiles/READ_IMU.dir/flags.make
CMakeFiles/READ_IMU.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ikon/rt_imu_data_cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/READ_IMU.dir/main.cpp.o"
	/usr/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/READ_IMU.dir/main.cpp.o -c /home/ikon/rt_imu_data_cpp/main.cpp

CMakeFiles/READ_IMU.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/READ_IMU.dir/main.cpp.i"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ikon/rt_imu_data_cpp/main.cpp > CMakeFiles/READ_IMU.dir/main.cpp.i

CMakeFiles/READ_IMU.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/READ_IMU.dir/main.cpp.s"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ikon/rt_imu_data_cpp/main.cpp -o CMakeFiles/READ_IMU.dir/main.cpp.s

CMakeFiles/READ_IMU.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/READ_IMU.dir/main.cpp.o.requires

CMakeFiles/READ_IMU.dir/main.cpp.o.provides: CMakeFiles/READ_IMU.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/READ_IMU.dir/build.make CMakeFiles/READ_IMU.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/READ_IMU.dir/main.cpp.o.provides

CMakeFiles/READ_IMU.dir/main.cpp.o.provides.build: CMakeFiles/READ_IMU.dir/main.cpp.o


CMakeFiles/READ_IMU.dir/imu.cpp.o: CMakeFiles/READ_IMU.dir/flags.make
CMakeFiles/READ_IMU.dir/imu.cpp.o: ../imu.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ikon/rt_imu_data_cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/READ_IMU.dir/imu.cpp.o"
	/usr/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/READ_IMU.dir/imu.cpp.o -c /home/ikon/rt_imu_data_cpp/imu.cpp

CMakeFiles/READ_IMU.dir/imu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/READ_IMU.dir/imu.cpp.i"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ikon/rt_imu_data_cpp/imu.cpp > CMakeFiles/READ_IMU.dir/imu.cpp.i

CMakeFiles/READ_IMU.dir/imu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/READ_IMU.dir/imu.cpp.s"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ikon/rt_imu_data_cpp/imu.cpp -o CMakeFiles/READ_IMU.dir/imu.cpp.s

CMakeFiles/READ_IMU.dir/imu.cpp.o.requires:

.PHONY : CMakeFiles/READ_IMU.dir/imu.cpp.o.requires

CMakeFiles/READ_IMU.dir/imu.cpp.o.provides: CMakeFiles/READ_IMU.dir/imu.cpp.o.requires
	$(MAKE) -f CMakeFiles/READ_IMU.dir/build.make CMakeFiles/READ_IMU.dir/imu.cpp.o.provides.build
.PHONY : CMakeFiles/READ_IMU.dir/imu.cpp.o.provides

CMakeFiles/READ_IMU.dir/imu.cpp.o.provides.build: CMakeFiles/READ_IMU.dir/imu.cpp.o


CMakeFiles/READ_IMU.dir/i2c.c.o: CMakeFiles/READ_IMU.dir/flags.make
CMakeFiles/READ_IMU.dir/i2c.c.o: ../i2c.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ikon/rt_imu_data_cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/READ_IMU.dir/i2c.c.o"
	/usr/bin/aarch64-linux-gnu-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/READ_IMU.dir/i2c.c.o   -c /home/ikon/rt_imu_data_cpp/i2c.c

CMakeFiles/READ_IMU.dir/i2c.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/READ_IMU.dir/i2c.c.i"
	/usr/bin/aarch64-linux-gnu-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ikon/rt_imu_data_cpp/i2c.c > CMakeFiles/READ_IMU.dir/i2c.c.i

CMakeFiles/READ_IMU.dir/i2c.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/READ_IMU.dir/i2c.c.s"
	/usr/bin/aarch64-linux-gnu-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ikon/rt_imu_data_cpp/i2c.c -o CMakeFiles/READ_IMU.dir/i2c.c.s

CMakeFiles/READ_IMU.dir/i2c.c.o.requires:

.PHONY : CMakeFiles/READ_IMU.dir/i2c.c.o.requires

CMakeFiles/READ_IMU.dir/i2c.c.o.provides: CMakeFiles/READ_IMU.dir/i2c.c.o.requires
	$(MAKE) -f CMakeFiles/READ_IMU.dir/build.make CMakeFiles/READ_IMU.dir/i2c.c.o.provides.build
.PHONY : CMakeFiles/READ_IMU.dir/i2c.c.o.provides

CMakeFiles/READ_IMU.dir/i2c.c.o.provides.build: CMakeFiles/READ_IMU.dir/i2c.c.o


# Object files for target READ_IMU
READ_IMU_OBJECTS = \
"CMakeFiles/READ_IMU.dir/main.cpp.o" \
"CMakeFiles/READ_IMU.dir/imu.cpp.o" \
"CMakeFiles/READ_IMU.dir/i2c.c.o"

# External object files for target READ_IMU
READ_IMU_EXTERNAL_OBJECTS =

READ_IMU: CMakeFiles/READ_IMU.dir/main.cpp.o
READ_IMU: CMakeFiles/READ_IMU.dir/imu.cpp.o
READ_IMU: CMakeFiles/READ_IMU.dir/i2c.c.o
READ_IMU: CMakeFiles/READ_IMU.dir/build.make
READ_IMU: CMakeFiles/READ_IMU.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ikon/rt_imu_data_cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable READ_IMU"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/READ_IMU.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/READ_IMU.dir/build: READ_IMU

.PHONY : CMakeFiles/READ_IMU.dir/build

CMakeFiles/READ_IMU.dir/requires: CMakeFiles/READ_IMU.dir/main.cpp.o.requires
CMakeFiles/READ_IMU.dir/requires: CMakeFiles/READ_IMU.dir/imu.cpp.o.requires
CMakeFiles/READ_IMU.dir/requires: CMakeFiles/READ_IMU.dir/i2c.c.o.requires

.PHONY : CMakeFiles/READ_IMU.dir/requires

CMakeFiles/READ_IMU.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/READ_IMU.dir/cmake_clean.cmake
.PHONY : CMakeFiles/READ_IMU.dir/clean

CMakeFiles/READ_IMU.dir/depend:
	cd /home/ikon/rt_imu_data_cpp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ikon/rt_imu_data_cpp /home/ikon/rt_imu_data_cpp /home/ikon/rt_imu_data_cpp/build /home/ikon/rt_imu_data_cpp/build /home/ikon/rt_imu_data_cpp/build/CMakeFiles/READ_IMU.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/READ_IMU.dir/depend

