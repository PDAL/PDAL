###############################################################################
#
# CMake module to search for libLAS library
#
# On success, the macro sets the following variables:
# LIBLAS_FOUND       = if the library found
# LIBLAS_LIBRARIES   = full path to the library
# LIBLAS_INCLUDE_DIR = where to find the library headers also defined,
#                       but not for general use are
# LIBLAS_LIBRARY     = where to find the PROJ.4 library.
# LIBLAS_VERSION     = version of library which was found, e.g. "1.2.5"
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
# Module source: http://github.com/mloskot/workshop/tree/master/cmake/
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#
###############################################################################
MESSAGE(STATUS "Searching for LibLAS ${LibLAS_FIND_VERSION}+ library")

IF(LIBLAS_INCLUDE_DIR)
  # Already in cache, be silent
  SET(LIBLAS_FIND_QUIETLY TRUE)
ENDIF()

IF(WIN32)
  SET(OSGEO4W_IMPORT_LIBRARY liblas)
  IF(DEFINED ENV{OSGEO4W_ROOT})
    SET(OSGEO4W_ROOT_DIR $ENV{OSGEO4W_ROOT})
    #MESSAGE(STATUS " FindLibLAS: trying OSGeo4W using environment variable OSGEO4W_ROOT=$ENV{OSGEO4W_ROOT}")
  ELSE()
    SET(OSGEO4W_ROOT_DIR c:/OSGeo4W)
    #MESSAGE(STATUS " FindLibLAS: trying OSGeo4W using default location OSGEO4W_ROOT=${OSGEO4W_ROOT_DIR}")
  ENDIF()
ENDIF()


FIND_PATH(LIBLAS_INCLUDE_DIR
  liblas.hpp
  PATH_PREFIXES liblas
  PATHS
  /usr/include
  /usr/local/include
  /tmp/lasjunk/include
  ${OSGEO4W_ROOT_DIR}/include)

if(WIN32)
    SET(LIBLAS_NAMES ${OSGEO4W_IMPORT_LIBRARY} liblas)
else()
    SET(LIBLAS_NAMES ${OSGEO4W_IMPORT_LIBRARY} las)
endif()

FIND_LIBRARY(LIBLAS_LIBRARY
  NAMES ${LIBLAS_NAMES}
  PATHS
  /usr/lib
  /usr/local/lib
  /tmp/lasjunk/lib
  ${OSGEO4W_ROOT_DIR}/lib)

IF(LIBLAS_FOUND)
  SET(LIBLAS_LIBRARIES ${LIBLAS_LIBRARY})
ENDIF()

IF(LIBLAS_INCLUDE_DIR)
  SET(LIBLAS_VERSION 0)

  SET(LIBLAS_VERSION_H "${LIBLAS_INCLUDE_DIR}/liblas/version.hpp")
  FILE(READ ${LIBLAS_VERSION_H} LIBLAS_VERSION_H_CONTENTS)

  IF (DEFINED LIBLAS_VERSION_H_CONTENTS)
  
    # string will be something like "106000", which is xyyzzz (x=major, y=minor, z=patch)
    string(REGEX REPLACE ".*#define[ \t]LIBLAS_VERSION[ \t]+([0-9]).*" "\\1" LIBLAS_VERSION_MAJOR "${LIBLAS_VERSION_H_CONTENTS}")
    string(REGEX REPLACE ".*#define[ \t]LIBLAS_VERSION[ \t]+[0-9]([0-9][0-9]).*" "\\1" LIBLAS_VERSION_MINOR "${LIBLAS_VERSION_H_CONTENTS}")
    string(REGEX REPLACE ".*#define[ \t]LIBLAS_VERSION[ \t]+[0-9][0-9][0-9]([0-9][0-9][0-9]).*"   "\\1" LIBLAS_VERSION_PATCH   "${LIBLAS_VERSION_H_CONTENTS}")
    #message(FATAL_ERROR "${LIBLAS_VERSION_MAJOR}.${LIBLAS_VERSION_MINOR}.${LIBLAS_VERSION_PATCH}")

    if(NOT ${LIBLAS_VERSION_MAJOR} MATCHES "[0-9]+")
      message(FATAL_ERROR "libLAS version parsing failed for LIBLAS_VERSION_MAJOR!")
    endif()
    if(NOT ${LIBLAS_VERSION_MINOR} MATCHES "[0-9]+")
      message(FATAL_ERROR "libLAS version parsing failed for LIBLAS_VERSION_MINOR!")
    endif()
    if(NOT ${LIBLAS_VERSION_PATCH} MATCHES "[0-9]+")
      message(FATAL_ERROR "libLAS version parsing failed for LIBLAS_VERSION_PATCH!")
    endif()


    SET(LIBLAS_VERSION "${LIBLAS_VERSION_MAJOR}.${LIBLAS_VERSION_MINOR}.${LIBLAS_VERSION_PATCH}"
      CACHE INTERNAL "The version string for libLAS library")

    IF (LIBLAS_VERSION VERSION_EQUAL LibLAS_FIND_VERSION OR
        LIBLAS_VERSION VERSION_GREATER LibLAS_FIND_VERSION)
      MESSAGE(STATUS "Found libLAS version: ${LIBLAS_VERSION}")
    ELSE()
      MESSAGE(FATAL_ERROR "libLS version check failed. Version ${LIBLAS_VERSION} was found, at least version ${LibLAS_FIND_VERSION} is required")
    ENDIF()
  ELSE()
    MESSAGE(FATAL_ERROR "Failed to open ${LIBLAS_VERSION_H} file")
  ENDIF()

ENDIF()

# Handle the QUIETLY and REQUIRED arguments and set LIBLAS_FOUND to TRUE
# if all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(libLAS DEFAULT_MSG LIBLAS_LIBRARY LIBLAS_INCLUDE_DIR)
