###############################################################################
#
# CMake module to search for LASzip library
#
# On success, the macro sets the following variables:
# LASZIP_FOUND       = if the library found
# LASZIP_LIBRARIES   = full path to the library
# LASZIP_INCLUDE_DIR = where to find the library headers also defined,
#                       but not for general use are
# LASZIP_LIBRARY     = where to find the laszip library.
# LASZIP_VERSION     = version of library which was found, e.g. "1.2.5"
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
# Module source: http://github.com/mloskot/workshop/tree/master/cmake/
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#
###############################################################################

IF(LASZIP_INCLUDE_DIR)
  # Already in cache, be silent
  SET(LASZIP_FIND_QUIETLY TRUE)
ENDIF()

IF(WIN32)
  SET(OSGEO4W_IMPORT_LIBRARY laszip)
  IF(DEFINED ENV{OSGEO4W_ROOT})
    SET(OSGEO4W_ROOT_DIR $ENV{OSGEO4W_ROOT})
    #MESSAGE(STATUS " FindLASzip: trying OSGeo4W using environment variable OSGEO4W_ROOT=$ENV{OSGEO4W_ROOT}")
  ELSE()
    SET(OSGEO4W_ROOT_DIR c:/OSGeo4W)
    #MESSAGE(STATUS " FindLASzip: trying OSGeo4W using default location OSGEO4W_ROOT=${OSGEO4W_ROOT_DIR}")
  ENDIF()
ENDIF()

FIND_PATH(LASZIP_INCLUDE_DIR
  laszip_api.h
  PATHS
  /usr/include/laszip
  /usr/local/include/laszip
  ${OSGEO4W_ROOT_DIR}/include/laszip
  /usr/include
  /usr/local/include
  ${OSGEO4W_ROOT_DIR}/include)

SET(LASZIP_NAMES ${OSGEO4W_IMPORT_LIBRARY} laszip)

FIND_LIBRARY(LASZIP_LIBRARY
  NAMES ${LASZIP_NAMES}
  PATHS
  /usr/lib
  /usr/local/lib
  ${OSGEO4W_ROOT_DIR}/lib)

# Handle the QUIETLY and REQUIRED arguments and set LASZIP_FOUND to TRUE
# if all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LASzip DEFAULT_MSG LASZIP_LIBRARY LASZIP_INCLUDE_DIR)

IF(LASZIP_FOUND)
  SET(LASZIP_LIBRARIES ${LASZIP_LIBRARY})
ENDIF()
