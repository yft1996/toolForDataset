# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /home/anshuai/yft/CLion/clion-2019.2.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/anshuai/yft/CLion/clion-2019.2.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/anshuai/yft/tool

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/anshuai/yft/tool/build

# Include any dependencies generated for this target.
include CMakeFiles/readColmapPose.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/readColmapPose.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/readColmapPose.dir/flags.make

CMakeFiles/readColmapPose.dir/src/colmap/readPose.cpp.o: CMakeFiles/readColmapPose.dir/flags.make
CMakeFiles/readColmapPose.dir/src/colmap/readPose.cpp.o: ../src/colmap/readPose.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/anshuai/yft/tool/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/readColmapPose.dir/src/colmap/readPose.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/readColmapPose.dir/src/colmap/readPose.cpp.o -c /home/anshuai/yft/tool/src/colmap/readPose.cpp

CMakeFiles/readColmapPose.dir/src/colmap/readPose.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/readColmapPose.dir/src/colmap/readPose.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/anshuai/yft/tool/src/colmap/readPose.cpp > CMakeFiles/readColmapPose.dir/src/colmap/readPose.cpp.i

CMakeFiles/readColmapPose.dir/src/colmap/readPose.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/readColmapPose.dir/src/colmap/readPose.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/anshuai/yft/tool/src/colmap/readPose.cpp -o CMakeFiles/readColmapPose.dir/src/colmap/readPose.cpp.s

# Object files for target readColmapPose
readColmapPose_OBJECTS = \
"CMakeFiles/readColmapPose.dir/src/colmap/readPose.cpp.o"

# External object files for target readColmapPose
readColmapPose_EXTERNAL_OBJECTS =

../bin/readColmapPose: CMakeFiles/readColmapPose.dir/src/colmap/readPose.cpp.o
../bin/readColmapPose: CMakeFiles/readColmapPose.dir/build.make
../bin/readColmapPose: CMakeFiles/readColmapPose.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/anshuai/yft/tool/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/readColmapPose"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/readColmapPose.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/readColmapPose.dir/build: ../bin/readColmapPose

.PHONY : CMakeFiles/readColmapPose.dir/build

CMakeFiles/readColmapPose.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/readColmapPose.dir/cmake_clean.cmake
.PHONY : CMakeFiles/readColmapPose.dir/clean

CMakeFiles/readColmapPose.dir/depend:
	cd /home/anshuai/yft/tool/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/anshuai/yft/tool /home/anshuai/yft/tool /home/anshuai/yft/tool/build /home/anshuai/yft/tool/build /home/anshuai/yft/tool/build/CMakeFiles/readColmapPose.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/readColmapPose.dir/depend

