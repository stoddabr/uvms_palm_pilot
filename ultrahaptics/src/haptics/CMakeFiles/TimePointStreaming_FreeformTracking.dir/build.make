# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/charisma/Documents/ultrahaptic/Examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/charisma/Documents/ultrahaptic/Examples/build

# Include any dependencies generated for this target.
include cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/compiler_depend.make

# Include the progress variables for this target.
include cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/progress.make

# Include the compile flags for this target's objects.
include cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/flags.make

cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/TimePointStreaming_FreeformTracking.cpp.o: cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/flags.make
cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/TimePointStreaming_FreeformTracking.cpp.o: ../cpp/TimePointStreaming_FreeformTracking.cpp
cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/TimePointStreaming_FreeformTracking.cpp.o: cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/charisma/Documents/ultrahaptic/Examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/TimePointStreaming_FreeformTracking.cpp.o"
	cd /home/charisma/Documents/ultrahaptic/Examples/build/cpp && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/TimePointStreaming_FreeformTracking.cpp.o -MF CMakeFiles/TimePointStreaming_FreeformTracking.dir/TimePointStreaming_FreeformTracking.cpp.o.d -o CMakeFiles/TimePointStreaming_FreeformTracking.dir/TimePointStreaming_FreeformTracking.cpp.o -c /home/charisma/Documents/ultrahaptic/Examples/cpp/TimePointStreaming_FreeformTracking.cpp

cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/TimePointStreaming_FreeformTracking.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TimePointStreaming_FreeformTracking.dir/TimePointStreaming_FreeformTracking.cpp.i"
	cd /home/charisma/Documents/ultrahaptic/Examples/build/cpp && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/charisma/Documents/ultrahaptic/Examples/cpp/TimePointStreaming_FreeformTracking.cpp > CMakeFiles/TimePointStreaming_FreeformTracking.dir/TimePointStreaming_FreeformTracking.cpp.i

cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/TimePointStreaming_FreeformTracking.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TimePointStreaming_FreeformTracking.dir/TimePointStreaming_FreeformTracking.cpp.s"
	cd /home/charisma/Documents/ultrahaptic/Examples/build/cpp && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/charisma/Documents/ultrahaptic/Examples/cpp/TimePointStreaming_FreeformTracking.cpp -o CMakeFiles/TimePointStreaming_FreeformTracking.dir/TimePointStreaming_FreeformTracking.cpp.s

# Object files for target TimePointStreaming_FreeformTracking
TimePointStreaming_FreeformTracking_OBJECTS = \
"CMakeFiles/TimePointStreaming_FreeformTracking.dir/TimePointStreaming_FreeformTracking.cpp.o"

# External object files for target TimePointStreaming_FreeformTracking
TimePointStreaming_FreeformTracking_EXTERNAL_OBJECTS =

cpp/TimePointStreaming_FreeformTracking: cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/TimePointStreaming_FreeformTracking.cpp.o
cpp/TimePointStreaming_FreeformTracking: cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/build.make
cpp/TimePointStreaming_FreeformTracking: /usr/lib/libUltrahaptics.so.2.6
cpp/TimePointStreaming_FreeformTracking: /home/charisma/Documents/LeapSDK/lib/x64/libLeap.so
cpp/TimePointStreaming_FreeformTracking: cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/charisma/Documents/ultrahaptic/Examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable TimePointStreaming_FreeformTracking"
	cd /home/charisma/Documents/ultrahaptic/Examples/build/cpp && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TimePointStreaming_FreeformTracking.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/build: cpp/TimePointStreaming_FreeformTracking
.PHONY : cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/build

cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/clean:
	cd /home/charisma/Documents/ultrahaptic/Examples/build/cpp && $(CMAKE_COMMAND) -P CMakeFiles/TimePointStreaming_FreeformTracking.dir/cmake_clean.cmake
.PHONY : cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/clean

cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/depend:
	cd /home/charisma/Documents/ultrahaptic/Examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/charisma/Documents/ultrahaptic/Examples /home/charisma/Documents/ultrahaptic/Examples/cpp /home/charisma/Documents/ultrahaptic/Examples/build /home/charisma/Documents/ultrahaptic/Examples/build/cpp /home/charisma/Documents/ultrahaptic/Examples/build/cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cpp/CMakeFiles/TimePointStreaming_FreeformTracking.dir/depend

