cmake_minimum_required(VERSION 3.11)
project(MPMbox)

cmake_policy (SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED COMPONENTS OpenGL)


#find_package(OpenGL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GLUT REQUIRED glut)

#find_package(GLUT REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../../common ${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR}) 
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
target_include_directories(see PUBLIC ${GLUT_INCLUDE_DIRS})
target_link_libraries (see PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone/libPBC3D.a ${OPENGL_LIBRARIES}   ${GLUT_LINK_LIBRARIES})
#target_link_libraries (see ${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone/libPBC3D.a ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

#add_executable(cut See/cut.cpp $<TARGET_OBJECTS:core_obj>)
#target_link_libraries (cut ${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone/libPBC3D.a ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})


ADD_DEFINITIONS(
-O3
-std=c++11
-Wno-deprecated-declarations
-Wall
-Wextra
)
