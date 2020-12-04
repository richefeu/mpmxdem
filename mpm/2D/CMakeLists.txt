cmake_minimum_required(VERSION 2.8)
project(MPMbox)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../../common)

#set(CMAKE_EXE_LINKER_FLAGS "-static-libstdc++")

file( GLOB core_object_files ${CMAKE_CURRENT_SOURCE_DIR}
BoundaryType/*.cpp
ConstitutiveModels/*.cpp
OneStep/*.cpp
Core/*.cpp
ShapeFunctions/*.cpp
Obstacles/*.cpp
Commands/*.cpp
Spies/*.cpp
VtkOutputs/*.cpp
)

file( GLOB trajecto_object_files ${CMAKE_CURRENT_SOURCE_DIR}
Trajectory/*.cpp
)

add_library( core_obj OBJECT ${core_object_files} )
add_executable( mpmbox Runners/run.cpp $<TARGET_OBJECTS:core_obj> )


#add_executable( mpmbox ${source_files} )
#target_link_libraries( mpmbox ${CMAKE_SOURCE_DIR}/../../OTHER/PBC3D/libPBC3D.a )

ADD_DEFINITIONS(
-O3
-std=c++11
-Wall
-Wextra
)
