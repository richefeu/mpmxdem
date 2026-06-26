# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/ab287450/Codes/mpmxdem/mpm/2Daxi/deps/toofus-src")
  file(MAKE_DIRECTORY "/home/ab287450/Codes/mpmxdem/mpm/2Daxi/deps/toofus-src")
endif()
file(MAKE_DIRECTORY
  "/home/ab287450/Codes/mpmxdem/mpm/2Daxi/deps/toofus-build"
  "/home/ab287450/Codes/mpmxdem/mpm/2Daxi/deps/toofus-subbuild/toofus-populate-prefix"
  "/home/ab287450/Codes/mpmxdem/mpm/2Daxi/deps/toofus-subbuild/toofus-populate-prefix/tmp"
  "/home/ab287450/Codes/mpmxdem/mpm/2Daxi/deps/toofus-subbuild/toofus-populate-prefix/src/toofus-populate-stamp"
  "/home/ab287450/Codes/mpmxdem/mpm/2Daxi/deps/toofus-subbuild/toofus-populate-prefix/src"
  "/home/ab287450/Codes/mpmxdem/mpm/2Daxi/deps/toofus-subbuild/toofus-populate-prefix/src/toofus-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/ab287450/Codes/mpmxdem/mpm/2Daxi/deps/toofus-subbuild/toofus-populate-prefix/src/toofus-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/ab287450/Codes/mpmxdem/mpm/2Daxi/deps/toofus-subbuild/toofus-populate-prefix/src/toofus-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
