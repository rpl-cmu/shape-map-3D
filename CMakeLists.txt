# cmake structure for GTSAM + MATLAB inspired from: https://github.com/gtrll/gpslam/blob/master/CMakeLists.txt

cmake_minimum_required(VERSION 3.0)
project(gpfactor CXX C)

set(GPFACTOR_VERSION_MAJOR 1)
set(GPFACTOR_VERSION_MINOR 0)
set(GPFACTOR_VERSION_PATCH 0)
set(gpfactor_VERSION_STRING "${GPSLAM_VERSION_MAJOR}.${GPSLAM_VERSION_MINOR}.${GPSLAM_VERSION_PATCH}")

# GTSAM-specific
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBS gtsam)

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

option(GPFACTOR_BUILD_TOOLBOX "Wrap and install matlab toolbox?" OFF)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Boost-specific
find_package(Boost 1.46 REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Boost COMPONENTS serialization REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(gpfactor)

# MATLAB wrap
if(GPFACTOR_BUILD_TOOLBOX)
  include(GtsamMatlabWrap)
  include_directories("/home/suddhu/packages/MATLAB/extern/include")
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})
  wrap_and_install_library(gpfactor.h ${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}" "")
endif()

GtsamMakeConfigFile(gpfactor)
export(TARGETS ${ggpfactor_EXPORTED_TARGETS} FILE gpfactor-exports.cmake)

