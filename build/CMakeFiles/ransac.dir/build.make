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
CMAKE_COMMAND = /home/ivashmak/clion-2018.1.2/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/ivashmak/clion-2018.1.2/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ivashmak/Ransac

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ivashmak/Ransac/build

# Include any dependencies generated for this target.
include CMakeFiles/ransac.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ransac.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ransac.dir/flags.make

CMakeFiles/ransac.dir/main.cpp.o: CMakeFiles/ransac.dir/flags.make
CMakeFiles/ransac.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivashmak/Ransac/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ransac.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ransac.dir/main.cpp.o -c /home/ivashmak/Ransac/main.cpp

CMakeFiles/ransac.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ransac.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ivashmak/Ransac/main.cpp > CMakeFiles/ransac.dir/main.cpp.i

CMakeFiles/ransac.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ransac.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ivashmak/Ransac/main.cpp -o CMakeFiles/ransac.dir/main.cpp.s

CMakeFiles/ransac.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/ransac.dir/main.cpp.o.requires

CMakeFiles/ransac.dir/main.cpp.o.provides: CMakeFiles/ransac.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/ransac.dir/build.make CMakeFiles/ransac.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/ransac.dir/main.cpp.o.provides

CMakeFiles/ransac.dir/main.cpp.o.provides.build: CMakeFiles/ransac.dir/main.cpp.o


CMakeFiles/ransac.dir/Generator/generator.cpp.o: CMakeFiles/ransac.dir/flags.make
CMakeFiles/ransac.dir/Generator/generator.cpp.o: ../Generator/generator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivashmak/Ransac/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ransac.dir/Generator/generator.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ransac.dir/Generator/generator.cpp.o -c /home/ivashmak/Ransac/Generator/generator.cpp

CMakeFiles/ransac.dir/Generator/generator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ransac.dir/Generator/generator.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ivashmak/Ransac/Generator/generator.cpp > CMakeFiles/ransac.dir/Generator/generator.cpp.i

CMakeFiles/ransac.dir/Generator/generator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ransac.dir/Generator/generator.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ivashmak/Ransac/Generator/generator.cpp -o CMakeFiles/ransac.dir/Generator/generator.cpp.s

CMakeFiles/ransac.dir/Generator/generator.cpp.o.requires:

.PHONY : CMakeFiles/ransac.dir/Generator/generator.cpp.o.requires

CMakeFiles/ransac.dir/Generator/generator.cpp.o.provides: CMakeFiles/ransac.dir/Generator/generator.cpp.o.requires
	$(MAKE) -f CMakeFiles/ransac.dir/build.make CMakeFiles/ransac.dir/Generator/generator.cpp.o.provides.build
.PHONY : CMakeFiles/ransac.dir/Generator/generator.cpp.o.provides

CMakeFiles/ransac.dir/Generator/generator.cpp.o.provides.build: CMakeFiles/ransac.dir/Generator/generator.cpp.o


CMakeFiles/ransac.dir/Detector/detector.cpp.o: CMakeFiles/ransac.dir/flags.make
CMakeFiles/ransac.dir/Detector/detector.cpp.o: ../Detector/detector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivashmak/Ransac/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/ransac.dir/Detector/detector.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ransac.dir/Detector/detector.cpp.o -c /home/ivashmak/Ransac/Detector/detector.cpp

CMakeFiles/ransac.dir/Detector/detector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ransac.dir/Detector/detector.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ivashmak/Ransac/Detector/detector.cpp > CMakeFiles/ransac.dir/Detector/detector.cpp.i

CMakeFiles/ransac.dir/Detector/detector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ransac.dir/Detector/detector.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ivashmak/Ransac/Detector/detector.cpp -o CMakeFiles/ransac.dir/Detector/detector.cpp.s

CMakeFiles/ransac.dir/Detector/detector.cpp.o.requires:

.PHONY : CMakeFiles/ransac.dir/Detector/detector.cpp.o.requires

CMakeFiles/ransac.dir/Detector/detector.cpp.o.provides: CMakeFiles/ransac.dir/Detector/detector.cpp.o.requires
	$(MAKE) -f CMakeFiles/ransac.dir/build.make CMakeFiles/ransac.dir/Detector/detector.cpp.o.provides.build
.PHONY : CMakeFiles/ransac.dir/Detector/detector.cpp.o.provides

CMakeFiles/ransac.dir/Detector/detector.cpp.o.provides.build: CMakeFiles/ransac.dir/Detector/detector.cpp.o


CMakeFiles/ransac.dir/Ransac/Ransac.cpp.o: CMakeFiles/ransac.dir/flags.make
CMakeFiles/ransac.dir/Ransac/Ransac.cpp.o: ../Ransac/Ransac.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivashmak/Ransac/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/ransac.dir/Ransac/Ransac.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ransac.dir/Ransac/Ransac.cpp.o -c /home/ivashmak/Ransac/Ransac/Ransac.cpp

CMakeFiles/ransac.dir/Ransac/Ransac.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ransac.dir/Ransac/Ransac.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ivashmak/Ransac/Ransac/Ransac.cpp > CMakeFiles/ransac.dir/Ransac/Ransac.cpp.i

CMakeFiles/ransac.dir/Ransac/Ransac.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ransac.dir/Ransac/Ransac.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ivashmak/Ransac/Ransac/Ransac.cpp -o CMakeFiles/ransac.dir/Ransac/Ransac.cpp.s

CMakeFiles/ransac.dir/Ransac/Ransac.cpp.o.requires:

.PHONY : CMakeFiles/ransac.dir/Ransac/Ransac.cpp.o.requires

CMakeFiles/ransac.dir/Ransac/Ransac.cpp.o.provides: CMakeFiles/ransac.dir/Ransac/Ransac.cpp.o.requires
	$(MAKE) -f CMakeFiles/ransac.dir/build.make CMakeFiles/ransac.dir/Ransac/Ransac.cpp.o.provides.build
.PHONY : CMakeFiles/ransac.dir/Ransac/Ransac.cpp.o.provides

CMakeFiles/ransac.dir/Ransac/Ransac.cpp.o.provides.build: CMakeFiles/ransac.dir/Ransac/Ransac.cpp.o


CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.o: CMakeFiles/ransac.dir/flags.make
CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.o: ../Detector/ReadPoints.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivashmak/Ransac/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.o -c /home/ivashmak/Ransac/Detector/ReadPoints.cpp

CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ivashmak/Ransac/Detector/ReadPoints.cpp > CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.i

CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ivashmak/Ransac/Detector/ReadPoints.cpp -o CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.s

CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.o.requires:

.PHONY : CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.o.requires

CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.o.provides: CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.o.requires
	$(MAKE) -f CMakeFiles/ransac.dir/build.make CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.o.provides.build
.PHONY : CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.o.provides

CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.o.provides.build: CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.o


CMakeFiles/ransac.dir/Ransac/DLT.cpp.o: CMakeFiles/ransac.dir/flags.make
CMakeFiles/ransac.dir/Ransac/DLT.cpp.o: ../Ransac/DLT.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivashmak/Ransac/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/ransac.dir/Ransac/DLT.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ransac.dir/Ransac/DLT.cpp.o -c /home/ivashmak/Ransac/Ransac/DLT.cpp

CMakeFiles/ransac.dir/Ransac/DLT.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ransac.dir/Ransac/DLT.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ivashmak/Ransac/Ransac/DLT.cpp > CMakeFiles/ransac.dir/Ransac/DLT.cpp.i

CMakeFiles/ransac.dir/Ransac/DLT.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ransac.dir/Ransac/DLT.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ivashmak/Ransac/Ransac/DLT.cpp -o CMakeFiles/ransac.dir/Ransac/DLT.cpp.s

CMakeFiles/ransac.dir/Ransac/DLT.cpp.o.requires:

.PHONY : CMakeFiles/ransac.dir/Ransac/DLT.cpp.o.requires

CMakeFiles/ransac.dir/Ransac/DLT.cpp.o.provides: CMakeFiles/ransac.dir/Ransac/DLT.cpp.o.requires
	$(MAKE) -f CMakeFiles/ransac.dir/build.make CMakeFiles/ransac.dir/Ransac/DLT.cpp.o.provides.build
.PHONY : CMakeFiles/ransac.dir/Ransac/DLT.cpp.o.provides

CMakeFiles/ransac.dir/Ransac/DLT.cpp.o.provides.build: CMakeFiles/ransac.dir/Ransac/DLT.cpp.o


CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.o: CMakeFiles/ransac.dir/flags.make
CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.o: ../Ransac/NormalizedDLT.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivashmak/Ransac/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.o -c /home/ivashmak/Ransac/Ransac/NormalizedDLT.cpp

CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ivashmak/Ransac/Ransac/NormalizedDLT.cpp > CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.i

CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ivashmak/Ransac/Ransac/NormalizedDLT.cpp -o CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.s

CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.o.requires:

.PHONY : CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.o.requires

CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.o.provides: CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.o.requires
	$(MAKE) -f CMakeFiles/ransac.dir/build.make CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.o.provides.build
.PHONY : CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.o.provides

CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.o.provides.build: CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.o


CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.o: CMakeFiles/ransac.dir/flags.make
CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.o: ../Ransac/GetNormalizingTransformation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivashmak/Ransac/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.o -c /home/ivashmak/Ransac/Ransac/GetNormalizingTransformation.cpp

CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ivashmak/Ransac/Ransac/GetNormalizingTransformation.cpp > CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.i

CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ivashmak/Ransac/Ransac/GetNormalizingTransformation.cpp -o CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.s

CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.o.requires:

.PHONY : CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.o.requires

CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.o.provides: CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.o.requires
	$(MAKE) -f CMakeFiles/ransac.dir/build.make CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.o.provides.build
.PHONY : CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.o.provides

CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.o.provides.build: CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.o


CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.o: CMakeFiles/ransac.dir/flags.make
CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.o: ../tests/testHomographyFitting.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivashmak/Ransac/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.o -c /home/ivashmak/Ransac/tests/testHomographyFitting.cpp

CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ivashmak/Ransac/tests/testHomographyFitting.cpp > CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.i

CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ivashmak/Ransac/tests/testHomographyFitting.cpp -o CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.s

CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.o.requires:

.PHONY : CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.o.requires

CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.o.provides: CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.o.requires
	$(MAKE) -f CMakeFiles/ransac.dir/build.make CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.o.provides.build
.PHONY : CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.o.provides

CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.o.provides.build: CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.o


CMakeFiles/ransac.dir/tests/testLineFitting.cpp.o: CMakeFiles/ransac.dir/flags.make
CMakeFiles/ransac.dir/tests/testLineFitting.cpp.o: ../tests/testLineFitting.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivashmak/Ransac/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/ransac.dir/tests/testLineFitting.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ransac.dir/tests/testLineFitting.cpp.o -c /home/ivashmak/Ransac/tests/testLineFitting.cpp

CMakeFiles/ransac.dir/tests/testLineFitting.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ransac.dir/tests/testLineFitting.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ivashmak/Ransac/tests/testLineFitting.cpp > CMakeFiles/ransac.dir/tests/testLineFitting.cpp.i

CMakeFiles/ransac.dir/tests/testLineFitting.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ransac.dir/tests/testLineFitting.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ivashmak/Ransac/tests/testLineFitting.cpp -o CMakeFiles/ransac.dir/tests/testLineFitting.cpp.s

CMakeFiles/ransac.dir/tests/testLineFitting.cpp.o.requires:

.PHONY : CMakeFiles/ransac.dir/tests/testLineFitting.cpp.o.requires

CMakeFiles/ransac.dir/tests/testLineFitting.cpp.o.provides: CMakeFiles/ransac.dir/tests/testLineFitting.cpp.o.requires
	$(MAKE) -f CMakeFiles/ransac.dir/build.make CMakeFiles/ransac.dir/tests/testLineFitting.cpp.o.provides.build
.PHONY : CMakeFiles/ransac.dir/tests/testLineFitting.cpp.o.provides

CMakeFiles/ransac.dir/tests/testLineFitting.cpp.o.provides.build: CMakeFiles/ransac.dir/tests/testLineFitting.cpp.o


# Object files for target ransac
ransac_OBJECTS = \
"CMakeFiles/ransac.dir/main.cpp.o" \
"CMakeFiles/ransac.dir/Generator/generator.cpp.o" \
"CMakeFiles/ransac.dir/Detector/detector.cpp.o" \
"CMakeFiles/ransac.dir/Ransac/Ransac.cpp.o" \
"CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.o" \
"CMakeFiles/ransac.dir/Ransac/DLT.cpp.o" \
"CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.o" \
"CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.o" \
"CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.o" \
"CMakeFiles/ransac.dir/tests/testLineFitting.cpp.o"

# External object files for target ransac
ransac_EXTERNAL_OBJECTS =

ransac: CMakeFiles/ransac.dir/main.cpp.o
ransac: CMakeFiles/ransac.dir/Generator/generator.cpp.o
ransac: CMakeFiles/ransac.dir/Detector/detector.cpp.o
ransac: CMakeFiles/ransac.dir/Ransac/Ransac.cpp.o
ransac: CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.o
ransac: CMakeFiles/ransac.dir/Ransac/DLT.cpp.o
ransac: CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.o
ransac: CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.o
ransac: CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.o
ransac: CMakeFiles/ransac.dir/tests/testLineFitting.cpp.o
ransac: CMakeFiles/ransac.dir/build.make
ransac: /usr/local/lib/libopencv_stitching.so.3.3.0
ransac: /usr/local/lib/libopencv_superres.so.3.3.0
ransac: /usr/local/lib/libopencv_videostab.so.3.3.0
ransac: /usr/local/lib/libopencv_aruco.so.3.3.0
ransac: /usr/local/lib/libopencv_bgsegm.so.3.3.0
ransac: /usr/local/lib/libopencv_bioinspired.so.3.3.0
ransac: /usr/local/lib/libopencv_ccalib.so.3.3.0
ransac: /usr/local/lib/libopencv_dpm.so.3.3.0
ransac: /usr/local/lib/libopencv_face.so.3.3.0
ransac: /usr/local/lib/libopencv_freetype.so.3.3.0
ransac: /usr/local/lib/libopencv_fuzzy.so.3.3.0
ransac: /usr/local/lib/libopencv_hdf.so.3.3.0
ransac: /usr/local/lib/libopencv_img_hash.so.3.3.0
ransac: /usr/local/lib/libopencv_line_descriptor.so.3.3.0
ransac: /usr/local/lib/libopencv_optflow.so.3.3.0
ransac: /usr/local/lib/libopencv_reg.so.3.3.0
ransac: /usr/local/lib/libopencv_rgbd.so.3.3.0
ransac: /usr/local/lib/libopencv_saliency.so.3.3.0
ransac: /usr/local/lib/libopencv_stereo.so.3.3.0
ransac: /usr/local/lib/libopencv_structured_light.so.3.3.0
ransac: /usr/local/lib/libopencv_surface_matching.so.3.3.0
ransac: /usr/local/lib/libopencv_tracking.so.3.3.0
ransac: /usr/local/lib/libopencv_xfeatures2d.so.3.3.0
ransac: /usr/local/lib/libopencv_ximgproc.so.3.3.0
ransac: /usr/local/lib/libopencv_xobjdetect.so.3.3.0
ransac: /usr/local/lib/libopencv_xphoto.so.3.3.0
ransac: /usr/local/lib/libopencv_shape.so.3.3.0
ransac: /usr/local/lib/libopencv_photo.so.3.3.0
ransac: /usr/local/lib/libopencv_calib3d.so.3.3.0
ransac: /usr/local/lib/libopencv_phase_unwrapping.so.3.3.0
ransac: /usr/local/lib/libopencv_dnn.so.3.3.0
ransac: /usr/local/lib/libopencv_video.so.3.3.0
ransac: /usr/local/lib/libopencv_datasets.so.3.3.0
ransac: /usr/local/lib/libopencv_plot.so.3.3.0
ransac: /usr/local/lib/libopencv_text.so.3.3.0
ransac: /usr/local/lib/libopencv_features2d.so.3.3.0
ransac: /usr/local/lib/libopencv_flann.so.3.3.0
ransac: /usr/local/lib/libopencv_highgui.so.3.3.0
ransac: /usr/local/lib/libopencv_ml.so.3.3.0
ransac: /usr/local/lib/libopencv_videoio.so.3.3.0
ransac: /usr/local/lib/libopencv_imgcodecs.so.3.3.0
ransac: /usr/local/lib/libopencv_objdetect.so.3.3.0
ransac: /usr/local/lib/libopencv_imgproc.so.3.3.0
ransac: /usr/local/lib/libopencv_core.so.3.3.0
ransac: CMakeFiles/ransac.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ivashmak/Ransac/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Linking CXX executable ransac"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ransac.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ransac.dir/build: ransac

.PHONY : CMakeFiles/ransac.dir/build

CMakeFiles/ransac.dir/requires: CMakeFiles/ransac.dir/main.cpp.o.requires
CMakeFiles/ransac.dir/requires: CMakeFiles/ransac.dir/Generator/generator.cpp.o.requires
CMakeFiles/ransac.dir/requires: CMakeFiles/ransac.dir/Detector/detector.cpp.o.requires
CMakeFiles/ransac.dir/requires: CMakeFiles/ransac.dir/Ransac/Ransac.cpp.o.requires
CMakeFiles/ransac.dir/requires: CMakeFiles/ransac.dir/Detector/ReadPoints.cpp.o.requires
CMakeFiles/ransac.dir/requires: CMakeFiles/ransac.dir/Ransac/DLT.cpp.o.requires
CMakeFiles/ransac.dir/requires: CMakeFiles/ransac.dir/Ransac/NormalizedDLT.cpp.o.requires
CMakeFiles/ransac.dir/requires: CMakeFiles/ransac.dir/Ransac/GetNormalizingTransformation.cpp.o.requires
CMakeFiles/ransac.dir/requires: CMakeFiles/ransac.dir/tests/testHomographyFitting.cpp.o.requires
CMakeFiles/ransac.dir/requires: CMakeFiles/ransac.dir/tests/testLineFitting.cpp.o.requires

.PHONY : CMakeFiles/ransac.dir/requires

CMakeFiles/ransac.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ransac.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ransac.dir/clean

CMakeFiles/ransac.dir/depend:
	cd /home/ivashmak/Ransac/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ivashmak/Ransac /home/ivashmak/Ransac /home/ivashmak/Ransac/build /home/ivashmak/Ransac/build /home/ivashmak/Ransac/build/CMakeFiles/ransac.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ransac.dir/depend

