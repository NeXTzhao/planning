# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

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
CMAKE_COMMAND = /home/vtd/clion-2023.1.4/bin/cmake/linux/x64/bin/cmake

# The command to remove a file.
RM = /home/vtd/clion-2023.1.4/bin/cmake/linux/x64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/release_build

# Include any dependencies generated for this target.
include CMakeFiles/r_fun.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/r_fun.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/r_fun.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/r_fun.dir/flags.make

CMakeFiles/r_fun.dir/r_function.cpp.o: CMakeFiles/r_fun.dir/flags.make
CMakeFiles/r_fun.dir/r_function.cpp.o: /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/r_function.cpp
CMakeFiles/r_fun.dir/r_function.cpp.o: CMakeFiles/r_fun.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/release_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/r_fun.dir/r_function.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/r_fun.dir/r_function.cpp.o -MF CMakeFiles/r_fun.dir/r_function.cpp.o.d -o CMakeFiles/r_fun.dir/r_function.cpp.o -c /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/r_function.cpp

CMakeFiles/r_fun.dir/r_function.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/r_fun.dir/r_function.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/r_function.cpp > CMakeFiles/r_fun.dir/r_function.cpp.i

CMakeFiles/r_fun.dir/r_function.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/r_fun.dir/r_function.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/r_function.cpp -o CMakeFiles/r_fun.dir/r_function.cpp.s

CMakeFiles/r_fun.dir/r_function_test.cpp.o: CMakeFiles/r_fun.dir/flags.make
CMakeFiles/r_fun.dir/r_function_test.cpp.o: /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/r_function_test.cpp
CMakeFiles/r_fun.dir/r_function_test.cpp.o: CMakeFiles/r_fun.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/release_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/r_fun.dir/r_function_test.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/r_fun.dir/r_function_test.cpp.o -MF CMakeFiles/r_fun.dir/r_function_test.cpp.o.d -o CMakeFiles/r_fun.dir/r_function_test.cpp.o -c /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/r_function_test.cpp

CMakeFiles/r_fun.dir/r_function_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/r_fun.dir/r_function_test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/r_function_test.cpp > CMakeFiles/r_fun.dir/r_function_test.cpp.i

CMakeFiles/r_fun.dir/r_function_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/r_fun.dir/r_function_test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/r_function_test.cpp -o CMakeFiles/r_fun.dir/r_function_test.cpp.s

CMakeFiles/r_fun.dir/implicit_curve.cpp.o: CMakeFiles/r_fun.dir/flags.make
CMakeFiles/r_fun.dir/implicit_curve.cpp.o: /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/implicit_curve.cpp
CMakeFiles/r_fun.dir/implicit_curve.cpp.o: CMakeFiles/r_fun.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/release_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/r_fun.dir/implicit_curve.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/r_fun.dir/implicit_curve.cpp.o -MF CMakeFiles/r_fun.dir/implicit_curve.cpp.o.d -o CMakeFiles/r_fun.dir/implicit_curve.cpp.o -c /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/implicit_curve.cpp

CMakeFiles/r_fun.dir/implicit_curve.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/r_fun.dir/implicit_curve.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/implicit_curve.cpp > CMakeFiles/r_fun.dir/implicit_curve.cpp.i

CMakeFiles/r_fun.dir/implicit_curve.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/r_fun.dir/implicit_curve.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/implicit_curve.cpp -o CMakeFiles/r_fun.dir/implicit_curve.cpp.s

CMakeFiles/r_fun.dir/bezier_to_poly.cpp.o: CMakeFiles/r_fun.dir/flags.make
CMakeFiles/r_fun.dir/bezier_to_poly.cpp.o: /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/bezier_to_poly.cpp
CMakeFiles/r_fun.dir/bezier_to_poly.cpp.o: CMakeFiles/r_fun.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/release_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/r_fun.dir/bezier_to_poly.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/r_fun.dir/bezier_to_poly.cpp.o -MF CMakeFiles/r_fun.dir/bezier_to_poly.cpp.o.d -o CMakeFiles/r_fun.dir/bezier_to_poly.cpp.o -c /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/bezier_to_poly.cpp

CMakeFiles/r_fun.dir/bezier_to_poly.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/r_fun.dir/bezier_to_poly.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/bezier_to_poly.cpp > CMakeFiles/r_fun.dir/bezier_to_poly.cpp.i

CMakeFiles/r_fun.dir/bezier_to_poly.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/r_fun.dir/bezier_to_poly.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/bezier_to_poly.cpp -o CMakeFiles/r_fun.dir/bezier_to_poly.cpp.s

# Object files for target r_fun
r_fun_OBJECTS = \
"CMakeFiles/r_fun.dir/r_function.cpp.o" \
"CMakeFiles/r_fun.dir/r_function_test.cpp.o" \
"CMakeFiles/r_fun.dir/implicit_curve.cpp.o" \
"CMakeFiles/r_fun.dir/bezier_to_poly.cpp.o"

# External object files for target r_fun
r_fun_EXTERNAL_OBJECTS =

r_fun: CMakeFiles/r_fun.dir/r_function.cpp.o
r_fun: CMakeFiles/r_fun.dir/r_function_test.cpp.o
r_fun: CMakeFiles/r_fun.dir/implicit_curve.cpp.o
r_fun: CMakeFiles/r_fun.dir/bezier_to_poly.cpp.o
r_fun: CMakeFiles/r_fun.dir/build.make
r_fun: /usr/lib/x86_64-linux-gnu/libpython3.8.so
r_fun: CMakeFiles/r_fun.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/release_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable r_fun"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/r_fun.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/r_fun.dir/build: r_fun
.PHONY : CMakeFiles/r_fun.dir/build

CMakeFiles/r_fun.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/r_fun.dir/cmake_clean.cmake
.PHONY : CMakeFiles/r_fun.dir/clean

CMakeFiles/r_fun.dir/depend:
	cd /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/release_build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/release_build /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/release_build /home/vtd/Documents/planning/piecewise_bezier_sdf/c++_version/release_build/CMakeFiles/r_fun.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/r_fun.dir/depend
