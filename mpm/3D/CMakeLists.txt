cmake_minimum_required(VERSION 2.8)
project(MPMbox)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../../common)

set(CMAKE_EXE_LINKER_FLAGS -pg )

file( GLOB source_files ${CMAKE_CURRENT_SOURCE_DIR}

Commands/*.cpp
ConstitutiveModels/*.cpp
Core/*.cpp
ShapeFunctions/*.cpp
Obstacles/*.cpp
OneStep/*.cpp
blender_Obstacles/*.cpp

Runners/run.cpp
)

add_executable( mpmbox ${source_files} )

ADD_DEFINITIONS(
	-O3
    -std=c++0x
	-Wall
	-Wextra
)

install (
	TARGETS mpmbox
	DESTINATION /usr/local/bin
)
