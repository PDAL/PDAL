###############################################################################
#
# CMake module to search for Points2Grid library
#
# On success, the macro sets the following variables:
# P2G_FOUND       = if the library found
# P2G_LIBRARIES   = full path to the library
# P2G_INCLUDE_DIR = where to find the library headers also defined,
#                       but not for general use are
# P2G_LIBRARY     = where to find the PROJ.4 library.
# P2G_VERSION     = version of library which was found, e.g. "1.2.5"
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
# Module source: http://github.com/mloskot/workshop/tree/master/cmake/
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#
###############################################################################

IF(P2G_INCLUDE_DIR)
  # Already in cache, be silent
  SET(P2G_FIND_QUIETLY TRUE)
ENDIF()

IF(WIN32)
  SET(OSGEO4W_IMPORT_LIBRARY points2grid)
  IF(DEFINED ENV{OSGEO4W_ROOT})
    SET(OSGEO4W_ROOT_DIR $ENV{OSGEO4W_ROOT})
    #MESSAGE(STATUS " FindPoints2Grid: trying OSGeo4W using environment variable OSGEO4W_ROOT=$ENV{OSGEO4W_ROOT}")
  ELSE()
    SET(OSGEO4W_ROOT_DIR c:/OSGeo4W64)
    #MESSAGE(STATUS " FindPoints2Grid: trying OSGeo4W using default location OSGEO4W_ROOT=${OSGEO4W_ROOT_DIR}")
  ENDIF()
ENDIF()


FIND_PATH(P2G_INCLUDE_DIR
  points2grid/config.h
  PATHS
  /usr/include
  /usr/local/include
  /tmp/lasjunk
  ${OSGEO4W_ROOT_DIR}/include
  )

if(WIN32)
    SET(P2G_NAMES ${OSGEO4W_IMPORT_LIBRARY} libpts2grd)
else()
    SET(P2G_NAMES ${OSGEO4W_IMPORT_LIBRARY} pts2grd)
endif()

FIND_LIBRARY(P2G_LIBRARY
  NAMES ${P2G_NAMES}
  PATHS
  /usr/lib
  /usr/local/lib
  /tmp/lasjunk/lib
  ${OSGEO4W_ROOT_DIR}/lib)

IF(P2G_FOUND)
  SET(P2G_LIBRARIES ${P2G_LIBRARY})
ENDIF()

IF (NOT P2G_INCLUDE_DIR)
  MESSAGE(FATAL_ERROR "Unable to find Points2Grid include directory")
endif()

IF(P2G_INCLUDE_DIR)
  SET(P2G_VERSION 0)
  
  SET(P2G_VERSION_H "${P2G_INCLUDE_DIR}/points2grid/config.h")
  FILE(READ ${P2G_VERSION_H} P2G_VERSION_H_CONTENTS)
  
  IF (DEFINED P2G_VERSION_H_CONTENTS)
  
    string(REGEX REPLACE ".*#define[ \t]P2G_VERSION_MAJOR[ \t]+([0-9]+).*" "\\1" P2G_VERSION_MAJOR "${P2G_VERSION_H_CONTENTS}")
    string(REGEX REPLACE ".*#define[ \t]P2G_VERSION_MINOR[ \t]+([0-9]+).*" "\\1" P2G_VERSION_MINOR "${P2G_VERSION_H_CONTENTS}")
    string(REGEX REPLACE ".*#define[ \t]P2G_VERSION_PATCH[ \t]+([0-9]+).*" "\\1" P2G_VERSION_PATCH "${P2G_VERSION_H_CONTENTS}")
    #message(FATAL_ERROR "${P2G_VERSION_MAJOR}.${P2G_VERSION_MINOR}.${P2G_VERSION_PATCH}")

    if(NOT ${P2G_VERSION_MAJOR} MATCHES "[0-9]+")
      message(FATAL_ERROR "Points2Grid version parsing failed for P2G_VERSION_MAJOR!")
    endif()
    if(NOT ${P2G_VERSION_MINOR} MATCHES "[0-9]+")
      message(FATAL_ERROR "Points2Grid version parsing failed for P2G_VERSION_MINOR!")
    endif()
    if(NOT ${P2G_VERSION_PATCH} MATCHES "[0-9]+")
      message(FATAL_ERROR "Points2Grid version parsing failed for P2G_VERSION_PATCH!")
    endif()


    SET(P2G_VERSION "${P2G_VERSION_MAJOR}.${P2G_VERSION_MINOR}.${P2G_VERSION_PATCH}"
      CACHE INTERNAL "The version string for Points2Grid library")

    IF (P2G_VERSION VERSION_LESS Points2Grid_FIND_VERSION)
      MESSAGE(FATAL_ERROR "Points2Grid version check failed. Version ${P2G_VERSION} was found, at least version ${Points2Grid_FIND_VERSION} is required")
    ENDIF()
  ELSE()
    MESSAGE(FATAL_ERROR "Failed to open ${P2G_VERSION_H} file")
  ENDIF()

ENDIF()

# Handle the QUIETLY and REQUIRED arguments and set P2G_FOUND to TRUE
# if all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(P2G DEFAULT_MSG P2G_LIBRARY P2G_INCLUDE_DIR)
