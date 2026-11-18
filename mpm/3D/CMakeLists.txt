cmake_minimum_required(VERSION 3.22)

project(MPMbox3D LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

# ##############################################################################
# toofus (already fetched when built from the root CMakeLists.txt)
include(FetchContent)
FetchContent_Declare(
  toofus
  GIT_REPOSITORY https://github.com/richefeu/toofus.git
  GIT_TAG main)
FetchContent_MakeAvailable(toofus)

# The sources still include the former 'common/' folder of the repository
# (e.g. <../../common/factory.hpp>, "../../../common/stackTracer.hpp"), which
# has been replaced by toofus. A 'common' symlink to toofus is created in the
# build tree, and include directories are set so that these relative paths
# resolve to it.
set(COMMON_SHIM_DIR ${CMAKE_CURRENT_BINARY_DIR}/common_shim)
file(MAKE_DIRECTORY ${COMMON_SHIM_DIR}/a/b/c)
file(CREATE_LINK ${toofus_SOURCE_DIR} ${COMMON_SHIM_DIR}/common SYMBOLIC)

# ##############################################################################
# DEM library (target PBC3D), unless the root CMakeLists.txt already added it
if(NOT TARGET PBC3D)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone
                   ${CMAKE_CURRENT_BINARY_DIR}/PBC3D)
endif()

# ##############################################################################
# mpmbox (3D). The target is named 'mpmbox3D' so that it does not clash with the
# 2D 'mpmbox' target when both are built from the root; the file is 'mpmbox'.
file(GLOB mpm3d_sources CONFIGURE_DEPENDS
  Commands/*.cpp
  ConstitutiveModels/*.cpp
  Core/*.cpp
  ShapeFunctions/*.cpp
  Obstacles/*.cpp
  OneStep/*.cpp
  blender_Obstacles/*.cpp)

add_executable(mpmbox3D Runners/run.cpp ${mpm3d_sources})
set_target_properties(mpmbox3D PROPERTIES OUTPUT_NAME mpmbox)
target_include_directories(mpmbox3D PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${COMMON_SHIM_DIR}/a/b     # <../../common/...>
  ${COMMON_SHIM_DIR}/a/b/c)  # "../../../common/..."
target_compile_options(mpmbox3D PRIVATE -O3 -Wall -Wextra)
target_link_libraries(mpmbox3D PRIVATE PBC3D)

install(PROGRAMS $<TARGET_FILE:mpmbox3D> DESTINATION bin RENAME mpmbox3D)

# Installation of headers (sub-directory layout kept, e.g. Core/MPMbox.hpp)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/mpmxdem/mpm3D
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
        PATTERN ".vscode" EXCLUDE
        PATTERN "build" EXCLUDE
        PATTERN "Examples" EXCLUDE)
