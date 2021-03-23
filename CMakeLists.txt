###################################################################################
cmake_minimum_required(VERSION 3.0)
project(ecLqr CXX)

# Include GTSAM CMake tools
find_package(GTSAMCMakeTools)
include(GtsamBuildTypes) # Load build type flags and default to Debug mode
include(GtsamTesting)    # Easy functions for creating unit tests and scripts
include(GtsamMatlabWrap) # Automatic MATLAB wrapper generation

# Ensure that local folder is searched before library folders
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

###################################################################################
# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
# Note: Since Jan-2019, GTSAMConfig.cmake defines exported CMake targets
# that automatically do include the include_directories() without the need
# to call include_directories(), just target_link_libraries(NAME gtsam)
#include_directories(${GTSAM_INCLUDE_DIR})

###################################################################################
# Build static library from common sources
set(LIB_NAME ${PROJECT_NAME})
file(GLOB headers "src/*.h")
file(GLOB sources "src/*.cpp")
add_library(${LIB_NAME} SHARED ${headers} ${sources})
target_link_libraries(${LIB_NAME} gtsam)

# Install library
install(TARGETS ${LIB_NAME} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

###################################################################################
# Build tests (CMake tracks the dependecy to link with GTSAM through our project's static library)
gtsamAddTestsGlob("ecLqr" "tests/test*.cpp" "" "${LIB_NAME}")

###################################################################################
# Build scripts (CMake tracks the dependecy to link with GTSAM through our project's static library)
gtsamAddExamplesGlob("scripts/*.cpp" "" "${LIB_NAME}")

# ###################################################################################
# # Build MATLAB wrapper (CMake tracks the dependecy to link with GTSAM through our project's static library)
# wrap_and_install_library("ecLqr.h" "${LIB_NAME}" "" "")
