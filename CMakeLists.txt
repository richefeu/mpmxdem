cmake_minimum_required(VERSION 3.22)

project(MPMxDEM LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

option(MPMXDEM_BUILD_2D "Build MPM 2D (mpm/2D)" ON)
option(MPMXDEM_BUILD_3D "Build MPM 3D (mpm/3D)" ON)

# ##############################################################################
# toofus, fetched once and shared by all sub-projects.
# Use -DFETCHCONTENT_SOURCE_DIR_TOOFUS=/path/to/toofus to use a local copy.
include(FetchContent)
FetchContent_Declare(
  toofus
  GIT_REPOSITORY https://github.com/richefeu/toofus.git
  GIT_TAG main)
FetchContent_MakeAvailable(toofus)
message(STATUS "toofus: ${toofus_SOURCE_DIR}")

# ##############################################################################
# DEM library libPBC3D.a (target PBC3D), used by MPM 2D and 3D
add_subdirectory(dem/3D_sandstone)

# ##############################################################################
# MPM
if(MPMXDEM_BUILD_2D)
  add_subdirectory(mpm/2D)
endif()
if(MPMXDEM_BUILD_3D)
  add_subdirectory(mpm/3D)
endif()
