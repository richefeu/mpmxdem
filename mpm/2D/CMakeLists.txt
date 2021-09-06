cmake_minimum_required(VERSION 3.11)
project(MPMbox)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#--- Find GLUT
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)


#--- Find OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)




#find_package(OpenGL REQUIRED COMPONENTS OpenGL)

#find_package(OpenGL REQUIRED)

find_package(PkgConfig REQUIRED)
#pkg_check_modules(GLUT REQUIRED)
#pkg_check_modules(GLUT REQUIRED glut)
#find_package(GLUT REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../../common ${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone ${OpenGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone)


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

file( GLOB trajecto_object_files ${CMAKE_CURRENT_SOURCE_DIR}
Trajectory/*.cpp
)


add_library(core_obj OBJECT ${core_object_files})
add_executable(mpmbox Runners/run.cpp $<TARGET_OBJECTS:core_obj>)
target_link_libraries (mpmbox  ${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone/libPBC3D.a)

add_executable(see See/see.cpp $<TARGET_OBJECTS:core_obj>)
#target_include_directories(see PUBLIC ${GLUT_INCLUDE_DIRS})
#target_link_libraries (see PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone/libPBC3D.a OpenGL::GL GLUT::GLUT OpenGL::GLU)
target_link_libraries (see ${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone/libPBC3D.a ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

add_executable(cut See/cut.cpp $<TARGET_OBJECTS:core_obj>)
target_link_libraries (cut ${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone/libPBC3D.a)


ADD_DEFINITIONS(
-O3
-std=c++11
-Wno-deprecated-declarations
-Wall
-Wextra
)
