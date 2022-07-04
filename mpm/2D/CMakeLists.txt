cmake_minimum_required(VERSION 3.16)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(CMAKE_CXX_COMPILER "g++-11") # do not use AppleClang
endif()

project(MPMbox LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-O3 -Wno-deprecated-declarations -Wall -Wextra -DENABLE_PROFILING")
set(CMAKE_CXX_FLAGS_DEBUG "-g")

#find_package(spdlog CONFIG REQUIRED)
#message(STATUS "SPDLOG_FOUND = " ${SPDLOG_FOUND})
#message(STATUS "SPDLOG_INCLUDE_DIRS = " ${SPDLOG_INCLUDE_DIRS})

# ========= OpenGL
find_package(OpenGL REQUIRED)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

  # you might use: export CXX="g++-11"
  # before the find_package(OpenMP)
  find_package(OpenMP)

  # here, we do not uses find_package(glut) so that the version installed by apple is NOT used
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GLUT REQUIRED glut)
  set(GLUT_INCLUDE_DIR ${GLUT_INCLUDE_DIRS})
  set(GLUT_LIBRARIES "-L${GLUT_LIBDIR} -lglut")
  message(STATUS "GLUT_INCLUDE_DIR = " ${GLUT_INCLUDE_DIR})
  message(STATUS "GLUT_LIBRARIES = " ${GLUT_LIBRARIES})
  
else()
  
  find_package(OpenMP)
  find_package(GLUT REQUIRED)

endif()


# Remark:
# Before being able to compile, the file 'libPBC3D.a' needs to be already compiled
set(PLUGGED_DEM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone)
set(PLUGGED_DEM_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone/libPBC3D.a)
set(COMMON_INCLUDE_DIR ~/toofus)
message(STATUS "PLUGGED_DEM_INCLUDE_DIR = " ${PLUGGED_DEM_INCLUDE_DIR})
message(STATUS "PLUGGED_DEM_LIBRARY = " ${PLUGGED_DEM_LIBRARY})
message(STATUS "COMMON_INCLUDE_DIR = " ${COMMON_INCLUDE_DIR})
message(STATUS "OpenMP_FLAGS = "  ${OpenMP_CXX_FLAGS} )
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${COMMON_INCLUDE_DIR} ${PLUGGED_DEM_INCLUDE_DIR}) 
link_directories(${PLUGGED_DEM_INCLUDE_DIR}) 

file(GLOB core_object_files ${CMAKE_CURRENT_SOURCE_DIR}
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
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${OpenMP_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")
  message(STATUS "CMAKE_CXX_FLAGS = "  ${CMAKE_CXX_FLAGS})
  add_executable(mpmbox Runners/run.cpp $<TARGET_OBJECTS:core_obj>)
  target_link_libraries(mpmbox PUBLIC ${PLUGGED_DEM_LIBRARY} OpenMP::OpenMP_CXX)
else()
  add_executable(mpmbox Runners/run.cpp $<TARGET_OBJECTS:core_obj>)
	target_link_libraries(mpmbox PUBLIC ${PLUGGED_DEM_LIBRARY})
endif()

add_executable(see See/see.cpp $<TARGET_OBJECTS:core_obj>)
target_link_libraries (see PUBLIC ${PLUGGED_DEM_LIBRARY} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
target_include_directories(see PUBLIC ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})

add_executable(cut See/cut.cpp $<TARGET_OBJECTS:core_obj>)
target_link_libraries(cut PUBLIC ${PLUGGED_DEM_LIBRARY})


