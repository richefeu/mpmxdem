cmake_minimum_required(VERSION 3.22)

message(STATUS "CMAKE_SYSTEM_NAME = " ${CMAKE_SYSTEM_NAME})
message(STATUS "CMAKE_HOST_SYSTEM_NAME = " ${CMAKE_HOST_SYSTEM_NAME})

if(${CMAKE_HOST_SYSTEM_NAME} MATCHES "Darwin")
  # Try to find the latest GNU g++ installed via Homebrew
  execute_process(
    COMMAND
      bash -c
      "ls /usr/local/bin/g++-* /opt/homebrew/bin/g++-* 2>/dev/null | grep -Eo 'g\\+\\+-[0-9]+' | sort -V | tail -1"
    OUTPUT_VARIABLE LATEST_GNU_GPP
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(LATEST_GNU_GPP)
    set(CMAKE_OSX_COMPILER_ID "GNU")
    set(CMAKE_CXX_COMPILER "${LATEST_GNU_GPP}")
    message(STATUS "Using GNU g++: ${LATEST_GNU_GPP}")
  else()
    message(
      WARNING
        "No GNU g++ found in /usr/local/bin/ or /opt/homebrew/bin/. Falling back to default compiler."
    )
  endif()
endif()

project(MPMbox LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# -Wno-deprecated-declarations
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -DENABLE_PROFILING -lpthread")
set(CMAKE_CXX_FLAGS_DEBUG "-g")

include(FetchContent)
set(FETCHCONTENT_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps)

# ##############################################################################
# Fetching TOOFUS
FetchContent_Declare(
  toofus
  GIT_REPOSITORY https://github.com/richefeu/toofus.git
  GIT_TAG main)
FetchContent_GetProperties(toofus)
if(NOT toofus_POPULATED)
  message(STATUS "Fetching toofus")
  FetchContent_MakeAvailable(toofus)
endif()

# ##############################################################################
# OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP CXX has been found")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS
      "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# ##############################################################################
# OpenGL
# OpenGL and glut are only needed by the 'see' viewer, which is skipped
# when they are not found.
find_package(OpenGL)
if(OPENGL_FOUND)
  message(STATUS "OPENGL_INCLUDE_DIR = " ${OPENGL_INCLUDE_DIR})
  message(STATUS "OPENGL_LIBRARIES = " ${OPENGL_LIBRARIES})
endif()

# ##############################################################################
# glut (freeglut)
find_package(GLUT)
if(GLUT_FOUND)
  message(STATUS "GLUT_INCLUDE_DIR = " ${GLUT_INCLUDE_DIR})
  message(STATUS "GLUT_LIBRARIES = " ${GLUT_LIBRARIES})
endif()

# ##############################################################################
# mpmbox

# Remark: when built on its own, the file 'libPBC3D.a' needs to be already
# compiled. When built from the root CMakeLists.txt, the PBC3D target is used.
set(PLUGGED_DEM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone)
if(TARGET PBC3D)
  set(PLUGGED_DEM_LIBRARY PBC3D)
else()
  set(PLUGGED_DEM_LIBRARY
      ${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone/libPBC3D.a)
endif()

message(STATUS "PLUGGED_DEM_INCLUDE_DIR = " ${PLUGGED_DEM_INCLUDE_DIR})
message(STATUS "PLUGGED_DEM_LIBRARY = " ${PLUGGED_DEM_LIBRARY})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR} ${toofus_SOURCE_DIR} ${tclap_SOURCE_DIR}/include
  ${fmt_SOURCE_DIR}/include ${PLUGGED_DEM_INCLUDE_DIR})

link_directories(${PLUGGED_DEM_INCLUDE_DIR})

file(
  GLOB
  core_object_files
  CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}
  BoundaryForceLaw/*.cpp
  ConstitutiveModels/*.cpp
  OneStep/*.cpp
  Core/*.cpp
  ShapeFunctions/*.cpp
  Obstacles/*.cpp
  Commands/*.cpp
  Spies/*.cpp
  Schedulers/*.cpp)

# The static library that holds everything but the runners: 'libMPMbox.a'.
# mpmbox, see and mpmpost all link against it.
#
# The target is named 'mpmbox_lib' rather than 'MPMbox': macOS file systems
# are case-insensitive by default, so a target 'MPMbox' and the executable
# 'mpmbox' would fight over the same CMakeFiles/<target>.dir directory. Only
# the OUTPUT_NAME matters for the file that is produced.
add_library(mpmbox_lib STATIC ${core_object_files})
set_target_properties(mpmbox_lib PROPERTIES OUTPUT_NAME MPMbox)
target_link_libraries(mpmbox_lib PUBLIC ${PLUGGED_DEM_LIBRARY})

add_executable(mpmbox Runners/run.cpp)
target_link_libraries(mpmbox PUBLIC mpmbox_lib)


# ##############################################################################
# see
#
# On apple, compile freeglut from source with this configuration:
#
# mkdir build
# cd build
# cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DFREEGLUT_COCOA=ON -DFREEGLUT_BUILD_DEMOS=OFF -DCMAKE_BUILD_TYPE=Release
# make
# sudo make install

if(OPENGL_FOUND AND GLUT_FOUND)
  add_executable(see See/see.cpp)
  target_include_directories(see PRIVATE ${OPENGL_INCLUDE_DIR}
                                         ${GLUT_INCLUDE_DIRS} /usr/local/include)
  target_link_libraries(see PUBLIC mpmbox_lib OpenGL::GL OpenGL::GLU
                                   ${GLUT_LIBRARIES})
else()
  message(WARNING "OpenGL/GLUT not found: the 'see' viewer will not be built")
endif()


# ##############################################################################
# mpmpost
#
# Post-processing driven by a command file. See Doc/SyntaxMPMpost.md.
# The main is in Runners/, so that Post/*.cpp holds only the actions.

file(GLOB post_object_files CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}
     Post/*.cpp)

add_executable(mpmpost Runners/post.cpp ${post_object_files})
target_link_libraries(mpmpost PUBLIC mpmbox_lib)




# ##############################################################################
# Installation of headers (sub-directory layout kept, e.g. Core/MPMbox.hpp)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/mpmxdem/mpm2D
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
        PATTERN ".vscode" EXCLUDE
        PATTERN "build" EXCLUDE
        PATTERN "deps" EXCLUDE
        PATTERN "Examples" EXCLUDE
        PATTERN "Doc" EXCLUDE
        PATTERN "sphinxdoc" EXCLUDE
        PATTERN "Tests" EXCLUDE)
