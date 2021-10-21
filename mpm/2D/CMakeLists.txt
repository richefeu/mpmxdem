cmake_minimum_required(VERSION 3.16)
project(MPMbox LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-O3 -Wno-deprecated-declarations -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")

find_package(OpenMP REQUIRED)

find_package(OpenGL REQUIRED)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GLUT REQUIRED GLUT)
  set(GLUT_LIBRARIES ${GLUT_LINK_LIBRARIES})
  message(NOTICE "GLUT_INCLUDE_DIRS = " ${GLUT_INCLUDE_DIRS})
  message(NOTICE "GLUT_LIBRARIES = " ${GLUT_LIBRARIES})

else()

  find_package(GLUT REQUIRED)

endif()

# Remark:
# Before being able to compile, the file 'libPBC3D.a' needs to be already compiled
set(PLUGGED_DEM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone)
set(PLUGGED_DEM_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone/libPBC3D.a)
set(COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
message(NOTICE "PLUGGED_DEM_INCLUDE_DIR = " ${PLUGGED_DEM_INCLUDE_DIR})
message(NOTICE "PLUGGED_DEM_LIBRARY = " ${PLUGGED_DEM_LIBRARY})
message(NOTICE "COMMON_INCLUDE_DIR = " ${COMMON_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${COMMON_INCLUDE_DIR} ${PLUGGED_DEM_INCLUDE_DIR}) 
link_directories(${PLUGGED_DEM_INCLUDE_DIR})


file( GLOB core_object_files ${CMAKE_CURRENT_SOURCE_DIR}
BoundaryForceLaw/*.cpp
ConstitutiveModels/*.cpp
OneStep/*.cpp
Core/*.cpp
ShapeFunctions/*.cpp
Obstacles/*.cpp
Commands/*.cpp
Spies/*.cpp
)


add_library(core_obj OBJECT ${core_object_files})
add_executable(mpmbox Runners/run.cpp $<TARGET_OBJECTS:core_obj>)
target_link_libraries (mpmbox ${PLUGGED_DEM_LIBRARY})


add_executable(see See/see.cpp $<TARGET_OBJECTS:core_obj>)
target_include_directories(see PUBLIC ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIRS})
target_link_libraries (see PUBLIC ${PLUGGED_DEM_LIBRARY} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} OpenMP::OpenMP_CXX)


add_executable(cut See/cut.cpp $<TARGET_OBJECTS:core_obj>)
target_link_libraries (cut ${PLUGGED_DEM_LIBRARY})

