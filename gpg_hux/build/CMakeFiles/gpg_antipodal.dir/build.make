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
CMAKE_SOURCE_DIR = /home/tencent_go/learning/gpg

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tencent_go/learning/gpg/build

# Include any dependencies generated for this target.
include CMakeFiles/gpg_antipodal.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gpg_antipodal.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gpg_antipodal.dir/flags.make

CMakeFiles/gpg_antipodal.dir/src/gpg/antipodal.cpp.o: CMakeFiles/gpg_antipodal.dir/flags.make
CMakeFiles/gpg_antipodal.dir/src/gpg/antipodal.cpp.o: ../src/gpg/antipodal.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tencent_go/learning/gpg/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gpg_antipodal.dir/src/gpg/antipodal.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gpg_antipodal.dir/src/gpg/antipodal.cpp.o -c /home/tencent_go/learning/gpg/src/gpg/antipodal.cpp

CMakeFiles/gpg_antipodal.dir/src/gpg/antipodal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gpg_antipodal.dir/src/gpg/antipodal.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tencent_go/learning/gpg/src/gpg/antipodal.cpp > CMakeFiles/gpg_antipodal.dir/src/gpg/antipodal.cpp.i

CMakeFiles/gpg_antipodal.dir/src/gpg/antipodal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gpg_antipodal.dir/src/gpg/antipodal.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tencent_go/learning/gpg/src/gpg/antipodal.cpp -o CMakeFiles/gpg_antipodal.dir/src/gpg/antipodal.cpp.s

# Object files for target gpg_antipodal
gpg_antipodal_OBJECTS = \
"CMakeFiles/gpg_antipodal.dir/src/gpg/antipodal.cpp.o"

# External object files for target gpg_antipodal
gpg_antipodal_EXTERNAL_OBJECTS =

libgpg_antipodal.a: CMakeFiles/gpg_antipodal.dir/src/gpg/antipodal.cpp.o
libgpg_antipodal.a: CMakeFiles/gpg_antipodal.dir/build.make
libgpg_antipodal.a: CMakeFiles/gpg_antipodal.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tencent_go/learning/gpg/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libgpg_antipodal.a"
	$(CMAKE_COMMAND) -P CMakeFiles/gpg_antipodal.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gpg_antipodal.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gpg_antipodal.dir/build: libgpg_antipodal.a

.PHONY : CMakeFiles/gpg_antipodal.dir/build

CMakeFiles/gpg_antipodal.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gpg_antipodal.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gpg_antipodal.dir/clean

CMakeFiles/gpg_antipodal.dir/depend:
	cd /home/tencent_go/learning/gpg/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tencent_go/learning/gpg /home/tencent_go/learning/gpg /home/tencent_go/learning/gpg/build /home/tencent_go/learning/gpg/build /home/tencent_go/learning/gpg/build/CMakeFiles/gpg_antipodal.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gpg_antipodal.dir/depend

