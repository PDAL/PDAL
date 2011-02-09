###############################################################################
# CMake module to search for SpatialIndex library
#
# On success, the macro sets the following variables:
# SPATIALINDEX_FOUND       = if the library found
# SPATIALINDEX_LIBRARY     = full path to the library
# SPATIALINDEX_INCLUDE_DIR = where to find the library headers 
# SPATIALINDEX_VERSION     = version of library which was found, e.g. "1.4.0"
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#
###############################################################################

if(SPATIALINDEX_INCLUDE_DIR AND SPATIALINDEX_LIBRARY)
  # Already in cache, be silent
  set(SPATIALINDEX_FIND_QUIETLY TRUE)
else()
  message(STATUS "Searching for SpatialIndex ${SpatialIndex_FIND_VERSION}+ library")
endif()

if(WIN32)
  set(OSGEO4W_IMPORT_LIBRARY spatialindex_i)
  if(DEFINED ENV{OSGEO4W_ROOT})
    set(OSGEO4W_ROOT_DIR $ENV{OSGEO4W_ROOT})
    if(NOT SPATIALINDEX_FIND_QUIETLY)
      message(STATUS "Trying OSGeo4W using environment variable OSGEO4W_ROOT=$ENV{OSGEO4W_ROOT}")
    endif()
  else()
    set(OSGEO4W_ROOT_DIR c:/OSGeo4W)
    if(NOT SPATIALINDEX_FIND_QUIETLY)
      message(STATUS "Trying OSGeo4W using default location OSGEO4W_ROOT=${OSGEO4W_ROOT_DIR}")
    endif()
  endif()
endif()

find_path(SPATIALINDEX_INCLUDE_DIR
  NAMES MVRTree.h
  HINTS
  ${OSGEO4W_ROOT_DIR}/include
  PATHS
  ${OSGEO4W_ROOT_DIR}/include
  PATH_SUFFIXES spatialindex
  DOC "Path to include directory of SpatialIndex library")

set(SPATIALINDEX_NAMES ${OSGEO4W_IMPORT_LIBRARY} spatialindex)
find_library(SPATIALINDEX_LIBRARY
  NAMES ${SPATIALINDEX_NAMES}
  PATHS ${OSGEO4W_ROOT_DIR}/lib)

if (SPATIALINDEX_INCLUDE_DIR)
  set(SPATIALINDEX_VERSION 0)

  set(SPATIALINDEX_VERSION_H "${SPATIALINDEX_INCLUDE_DIR}/Version.h")
  if(NOT EXISTS ${SPATIALINDEX_VERSION_H})
    set(SPATIALINDEX_VERSION_H "${SPATIALINDEX_INCLUDE_DIR}/spatialindex/Version.h")
  endif()

  file(READ ${SPATIALINDEX_VERSION_H} SPATIALINDEX_VERSION_H_CONTENTS)

  if (DEFINED SPATIALINDEX_VERSION_H_CONTENTS)
    string(REGEX REPLACE ".*#define[ \t]SIDX_VERSION_MAJOR[ \t]+([0-9]+).*" "\\1" SIDX_VERSION_MAJOR "${SPATIALINDEX_VERSION_H_CONTENTS}")
    string(REGEX REPLACE ".*#define[ \t]SIDX_VERSION_MINOR[ \t]+([0-9]+).*" "\\1" SIDX_VERSION_MINOR "${SPATIALINDEX_VERSION_H_CONTENTS}")
    string(REGEX REPLACE ".*#define[ \t]SIDX_VERSION_REV[ \t]+([0-9]+).*"   "\\1" SIDX_VERSION_REV   "${SPATIALINDEX_VERSION_H_CONTENTS}")

    if(NOT ${SIDX_VERSION_MAJOR} MATCHES "[0-9]+")
      message(FATAL_ERROR "SpatialIndex version parsing failed for SIDX_VERSION_MAJOR!")
    endif()
    if(NOT ${SIDX_VERSION_MINOR} MATCHES "[0-9]+")
      message(FATAL_ERROR "SpatialIndex version parsing failed for SIDX_VERSION_MINOR!")
    endif()
    if(NOT ${SIDX_VERSION_REV} MATCHES "[0-9]+")
      message(FATAL_ERROR "SpatialIndex version parsing failed for SIDX_VERSION_REV!")
    endif()

    set(SPATIALINDEX_VERSION "${SIDX_VERSION_MAJOR}.${SIDX_VERSION_MINOR}.${SIDX_VERSION_REV}"
      CACHE INTERNAL "The version string for SpatialIndex library")

    if (SPATIALINDEX_FIND_QUIETLY AND
        (SPATIALINDEX_VERSION VERSION_EQUAL SpatialIndex_FIND_VERSION OR
          SPATIALINDEX_VERSION VERSION_GREATER SpatialIndex_FIND_VERSION))
      message(STATUS "Found SpatialIndex version: ${SPATIALINDEX_VERSION}")
    else()
        message(FATAL_ERROR "SpatialIndex version check failed. Version ${SPATIALINDEX_VERSION} was found, at least version ${SpatialIndex_FIND_VERSION} is required")
    endif()
  else()
      message(FATAL_ERROR "Failed to open ${SPATIALINDEX_VERSION_H} file")
  endif()

endif()

# Handle the QUIETLY and REQUIRED arguments and set SPATIALINDEX_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SpatialIndex DEFAULT_MSG SPATIALINDEX_LIBRARY SPATIALINDEX_INCLUDE_DIR)

# TODO: Do we want to mark these as advanced? --mloskot
# http://www.cmake.org/cmake/help/cmake2.6docs.html#command:mark_as_advanced
#MARK_AS_ADVANCED(SPATIALINDEX_LIBRARY SPATIALINDEX_INCLUDE_DIR)
