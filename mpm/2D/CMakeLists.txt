cmake_minimum_required(VERSION 3.3)

message(STATUS "CMAKE_SYSTEM_NAME = " ${CMAKE_SYSTEM_NAME})
message(STATUS "CMAKE_HOST_SYSTEM_NAME = " ${CMAKE_HOST_SYSTEM_NAME})
 
if (${CMAKE_HOST_SYSTEM_NAME} MATCHES "Darwin")
  set(CMAKE_OSX_COMPILER_ID "GNU")
  set(CMAKE_CXX_COMPILER "g++-14") # do not use AppleClang
  message(STATUS "CMAKE_CXX_COMPILER = " ${CMAKE_CXX_COMPILER})
  message(STATUS "On apple computers, you need to install the last g++ compiler (g++-14)")
endif()

project(MPMbox LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# -Wno-deprecated-declarations
set(CMAKE_CXX_FLAGS "-O3  -Wall -Wextra -DENABLE_PROFILING -lpthread")
set(CMAKE_CXX_FLAGS_DEBUG "-g")

include(FetchContent)
set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/deps)

######################################################################
##### Fetching TOOFUS
FetchContent_Declare (
	toofus
  GIT_REPOSITORY https://github.com/richefeu/toofus.git
  GIT_TAG        main
)
FetchContent_GetProperties(toofus)
if(NOT toofus_POPULATED)
  message(STATUS "Fetching toofus")
  FetchContent_Populate(toofus)
endif()

######################################################################
##### tclap
FetchContent_Declare (
	tclap
  GIT_REPOSITORY https://github.com/xguerin/tclap.git
  GIT_TAG        master
)
FetchContent_GetProperties(tclap)
if(NOT tclap_POPULATED)
  message(STATUS "Fetching tclap")
  FetchContent_Populate(tclap)
	#add_subdirectory(${tclap_SOURCE_DIR})
endif()

######################################################################
##### Fetching fmt
FetchContent_Declare(fmt
  GIT_REPOSITORY         https://github.com/fmtlib/fmt.git
  GIT_TAG                master
  GIT_PROGRESS           TRUE
  USES_TERMINAL_DOWNLOAD TRUE
)
FetchContent_GetProperties(fmt)
if(NOT fmt_POPULATED)
  message(STATUS "Fetching fmt")
  FetchContent_Populate(fmt)
  #add_subdirectory(${FETCHCONTENT_BASE_DIR}/fmt-src)
endif()



######################################################################
##### OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

######################################################################
##### OpenGL
find_package(OpenGL REQUIRED)
if(OPENGL_FOUND)
  message(STATUS "OPENGL_INCLUDE_DIR = " ${OPENGL_INCLUDE_DIR})
  message(STATUS "OPENGL_LIBRARIES = " ${OPENGL_LIBRARIES})
endif()

######################################################################
#### glut
if (${CMAKE_HOST_SYSTEM_NAME} MATCHES "Darwin")

  # here, we do not uses find_package(glut) so that the version installed by apple is NOT used
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GLUT REQUIRED glut>=3.4.0)
  IF(GLUT_FOUND)
    message(STATUS "GLUT as been found")
  ENDIF() 
  set(GLUT_INCLUDE_DIR ${GLUT_INCLUDE_DIRS})
  set(GLUT_LIBRARIES "-L${GLUT_LIBDIR} -lglut")
  message(STATUS "GLUT_INCLUDE_DIR = " ${GLUT_INCLUDE_DIR})
  message(STATUS "GLUT_LIBRARIES = " ${GLUT_LIBRARIES})
  
else()
  
  find_package(GLUT REQUIRED)

  message(STATUS "OPENGL_INCLUDE_DIR = " ${OPENGL_INCLUDE_DIR})
  message(STATUS "GLUT_INCLUDE_DIR = " ${GLUT_INCLUDE_DIR})
  message(STATUS "OPENGL_LIBRARIES = " ${OPENGL_LIBRARIES})
  message(STATUS "GLUT_LIBRARIES = " ${GLUT_LIBRARIES})
  
endif()

# Remark:
# Before being able to compile, the file 'libPBC3D.a' needs to be already compiled
set(PLUGGED_DEM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone)
set(PLUGGED_DEM_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/../../dem/3D_sandstone/libPBC3D.a)

message(STATUS "PLUGGED_DEM_INCLUDE_DIR = " ${PLUGGED_DEM_INCLUDE_DIR})
message(STATUS "PLUGGED_DEM_LIBRARY = " ${PLUGGED_DEM_LIBRARY})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR} 
  ${toofus_SOURCE_DIR}
  ${tclap_SOURCE_DIR}/include
  ${fmt_SOURCE_DIR}/include
  ${PLUGGED_DEM_INCLUDE_DIR}
) 

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
Schedulers/*.cpp
)

add_library(core_obj OBJECT ${core_object_files})

if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${OpenMP_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")
  message(STATUS "CMAKE_CXX_FLAGS = "  ${CMAKE_CXX_FLAGS})
	set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  add_executable(mpmbox Runners/run.cpp $<TARGET_OBJECTS:core_obj>)
  target_link_libraries(mpmbox PUBLIC ${PLUGGED_DEM_LIBRARY} OpenMP::OpenMP_CXX)
else()
  add_executable(mpmbox Runners/run.cpp $<TARGET_OBJECTS:core_obj>)
	target_link_libraries(mpmbox PUBLIC ${PLUGGED_DEM_LIBRARY})
endif()

add_executable(see See/see.cpp $<TARGET_OBJECTS:core_obj>)
target_link_libraries (see PUBLIC ${PLUGGED_DEM_LIBRARY} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
target_include_directories(see PUBLIC ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})

#add_executable(cut See/cut.cpp $<TARGET_OBJECTS:core_obj>)
#target_link_libraries(cut PUBLIC ${PLUGGED_DEM_LIBRARY})


