###############################################################################
#
# CMake module to search for Hexer library
#
# On success, the macro sets the following variables:
# HEXER_FOUND       = if the library found
# HEXER_LIBRARIES   = full path to the library
# HEXER_INCLUDE_DIR = where to find the library headers also defined,
#                       but not for general use are
# HEXER_LIBRARY     = where to find the hexer library.
# HEXER_VERSION     = version of library which was found, e.g. "1.2.5"
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#
###############################################################################

IF(HEXER_INCLUDE_DIR)
  # Already in cache, be silent
  SET(HEXER_FIND_QUIETLY TRUE)
ENDIF()

IF(WIN32)
  SET(OSGEO4W_IMPORT_LIBRARY hexer)
  IF(DEFINED ENV{OSGEO4W_ROOT})
    SET(OSGEO4W_ROOT_DIR $ENV{OSGEO4W_ROOT})
    #MESSAGE(STATUS " FindHexer: trying OSGeo4W using environment variable OSGEO4W_ROOT=$ENV{OSGEO4W_ROOT}")
  ELSE()
    SET(OSGEO4W_ROOT_DIR c:/OSGeo4W)
    #MESSAGE(STATUS " FindHexer: trying OSGeo4W using default location OSGEO4W_ROOT=${OSGEO4W_ROOT_DIR}")
  ENDIF()
ENDIF()


FIND_PATH(HEXER_INCLUDE_DIR
  hexer.hpp
  PATH_PREFIXES hexer
  PATHS
  /usr/include
  /usr/local/include
  /tmp/lasjunk/include
  ${OSGEO4W_ROOT_DIR}/include)

SET(HEXER_NAMES ${OSGEO4W_IMPORT_LIBRARY} hexer)

FIND_LIBRARY(HEXER_LIBRARY
  NAMES ${HEXER_NAMES}
  PATHS
  /usr/lib
  /usr/local/lib
  ${OSGEO4W_ROOT_DIR}/lib)

IF(HEXER_FOUND)
  SET(HEXER_LIBRARIES ${HEXER_LIBRARY})
ENDIF()

IF(HEXER_INCLUDE_DIR)
  SET(HEXER_VERSION 0)

  SET(HEXER_VERSION_H "${HEXER_INCLUDE_DIR}/hexer/hexer_defines.h")
  FILE(READ ${HEXER_VERSION_H} HEXER_VERSION_H_CONTENTS)

  IF (DEFINED HEXER_VERSION_H_CONTENTS)
    string(REGEX REPLACE ".*#define[ \t]HEXER_VERSION_MAJOR[ \t]+([0-9]+).*" "\\1" HEXER_VERSION_MAJOR "${HEXER_VERSION_H_CONTENTS}")
    string(REGEX REPLACE ".*#define[ \t]HEXER_VERSION_MINOR[ \t]+([0-9]+).*" "\\1" HEXER_VERSION_MINOR "${HEXER_VERSION_H_CONTENTS}")
    string(REGEX REPLACE ".*#define[ \t]HEXER_VERSION_PATCH[ \t]+([0-9]+).*"   "\\1" HEXER_VERSION_PATCH   "${HEXER_VERSION_H_CONTENTS}")

    if(NOT ${HEXER_VERSION_MAJOR} MATCHES "[0-9]+")
      message(FATAL_ERROR "Hexer version parsing failed for HEXER_VERSION_MAJOR!")
    endif()
    if(NOT ${HEXER_VERSION_MINOR} MATCHES "[0-9]+")
      message(FATAL_ERROR "Hexer version parsing failed for HEXER_VERSION_MINOR!")
    endif()
    if(NOT ${HEXER_VERSION_REVISION} MATCHES "[0-9]+")
      message(FATAL_ERROR "Hexer version parsing failed for HEXER_VERSION_PATCH!")
    endif()


    SET(HEXER_VERSION "${HEXER_VERSION_MAJOR}.${HEXER_VERSION_MINOR}.${HEXER_VERSION_PATCH}"
      CACHE INTERNAL "The version string for Hexer library")

    IF (HEXER_VERSION VERSION_LESS Hexer_FIND_VERSION)
      MESSAGE(FATAL_ERROR "Hexer version check failed. Version ${HEXER_VERSION} was found, at least version ${Hexer_FIND_VERSION} is required")
    ENDIF()
  ELSE()
    MESSAGE(FATAL_ERROR "Failed to open ${HEXER_VERSION_H} file")
  ENDIF()

ENDIF()

# Handle the QUIETLY and REQUIRED arguments and set HEXER_FOUND to TRUE
# if all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Hexer DEFAULT_MSG HEXER_LIBRARY HEXER_INCLUDE_DIR)
