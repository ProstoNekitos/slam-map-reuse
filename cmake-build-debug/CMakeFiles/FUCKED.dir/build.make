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
CMAKE_COMMAND = /usr/local/progz/clion-2019.3.2/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /usr/local/progz/clion-2019.3.2/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nolv/Workspace/map_reuse

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nolv/Workspace/map_reuse/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/FUCKED.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/FUCKED.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/FUCKED.dir/flags.make

CMakeFiles/FUCKED.dir/main.cpp.o: CMakeFiles/FUCKED.dir/flags.make
CMakeFiles/FUCKED.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nolv/Workspace/map_reuse/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/FUCKED.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FUCKED.dir/main.cpp.o -c /home/nolv/Workspace/map_reuse/main.cpp

CMakeFiles/FUCKED.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FUCKED.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nolv/Workspace/map_reuse/main.cpp > CMakeFiles/FUCKED.dir/main.cpp.i

CMakeFiles/FUCKED.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FUCKED.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nolv/Workspace/map_reuse/main.cpp -o CMakeFiles/FUCKED.dir/main.cpp.s

# Object files for target FUCKED
FUCKED_OBJECTS = \
"CMakeFiles/FUCKED.dir/main.cpp.o"

# External object files for target FUCKED
FUCKED_EXTERNAL_OBJECTS =

FUCKED: CMakeFiles/FUCKED.dir/main.cpp.o
FUCKED: CMakeFiles/FUCKED.dir/build.make
FUCKED: CMakeFiles/FUCKED.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nolv/Workspace/map_reuse/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable FUCKED"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FUCKED.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/FUCKED.dir/build: FUCKED

.PHONY : CMakeFiles/FUCKED.dir/build

CMakeFiles/FUCKED.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/FUCKED.dir/cmake_clean.cmake
.PHONY : CMakeFiles/FUCKED.dir/clean

CMakeFiles/FUCKED.dir/depend:
	cd /home/nolv/Workspace/map_reuse/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nolv/Workspace/map_reuse /home/nolv/Workspace/map_reuse /home/nolv/Workspace/map_reuse/cmake-build-debug /home/nolv/Workspace/map_reuse/cmake-build-debug /home/nolv/Workspace/map_reuse/cmake-build-debug/CMakeFiles/FUCKED.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/FUCKED.dir/depend

