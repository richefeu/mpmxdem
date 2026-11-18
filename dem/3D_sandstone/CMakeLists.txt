cmake_minimum_required(VERSION 3.22)

project(PBC3D LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ##############################################################################
# toofus (already fetched when built from the root CMakeLists.txt)
include(FetchContent)
FetchContent_Declare(
  toofus
  GIT_REPOSITORY https://github.com/richefeu/toofus.git
  GIT_TAG main)
FetchContent_MakeAvailable(toofus)

find_package(OpenMP REQUIRED)

# ##############################################################################
# libPBC3D.a: periodic 3D DEM, plugged into MPM 2D and 3D
add_library(PBC3D STATIC
  PBC3D.cpp
  Particle.cpp
  Interaction.cpp
  Loading.cpp
  PeriodicCell.cpp)
target_include_directories(PBC3D PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${toofus_SOURCE_DIR})
target_compile_options(PBC3D PRIVATE -O3 -Wall)
target_link_libraries(PBC3D PUBLIC OpenMP::OpenMP_CXX)

# ##############################################################################
# Installation of headers: PBC3D headers and the toofus headers they rely on
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/mpmxdem/PBC3D
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN ".vscode" EXCLUDE)
install(DIRECTORY ${toofus_SOURCE_DIR}/
        DESTINATION include/toofus
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
        PATTERN ".vscode" EXCLUDE
        PATTERN ".git" EXCLUDE
        PATTERN "examples" EXCLUDE
        PATTERN "doctests" EXCLUDE
        PATTERN "catch_unit_tests" EXCLUDE
        PATTERN "doc" EXCLUDE)
