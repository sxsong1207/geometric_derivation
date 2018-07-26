cmake_minimum_required(VERSION 2.8)

project(geometric_derivation)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/3rdparty/;${PROJECT_SOURCE_DIR}/3rdparty/lib/cmake;${CMAKE_PREFIX_PATH}")

## Determine toolchain
find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM)
    # Support Unix Makefiles and Ninja
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
    message(STATUS "USE CCACHE ${CCACHE_PROGRAM}")
endif (CCACHE_PROGRAM)

include(AutoBuildDependency)
if (BUILD_THIRD_PARTY)
    return()
endif (BUILD_THIRD_PARTY)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(3rdparty/include
        ${Sophus_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} $CERES_INCLUDE_DIRS})

add_subdirectory(quaternion)

