# ###############################################################################
# #
# # CMake module to search for Nitro library
# #
# # On success, the macro sets the following variables:
# # NITRO_FOUND       = if the library found
# # NITRO_LIBRARIES   = full path to the library
# # NITRO_INCLUDE_DIR = where to find the library headers also defined,
# #                       but not for general use are
# # NITRO_LIBRARY     = where to find the hexer library.
# # NITRO_VERSION     = version of library which was found, e.g. "1.2.5"
# #
# # Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
# #
# # Redistribution and use is allowed according to the terms of the BSD license.
# # For details see the accompanying COPYING-CMAKE-SCRIPTS file.
# #
# ###############################################################################

IF(NITRO_INCLUDE_DIR)
  # Already in cache, be silent
  SET(NITRO_FIND_QUIETLY TRUE)
ENDIF()

IF(WIN32)
  IF(DEFINED ENV{OSGEO4W_ROOT})
    SET(OSGEO4W_ROOT_DIR $ENV{OSGEO4W_ROOT})
  ELSE()
    SET(OSGEO4W_ROOT_DIR c:/OSGeo4W64)
  ENDIF()
ENDIF()

FIND_PATH(NITRO_INCLUDE_DIR
  nitro
  PATHS
  /usr/include
  /usr/local/include
  ${OSGEO4W_ROOT_DIR}/include)

SET(NITRO_C_NAME  nitf-c)

FIND_LIBRARY(NITRO_C_LIBRARY
  NAMES ${NITRO_C_NAME}
  PATHS
  /usr/lib
  /usr/local/lib
  ${OSGEO4W_ROOT_DIR}/lib)

SET(NITRO_CPP_NAME  nitf-cpp)

FIND_LIBRARY(NITRO_CPP_LIBRARY
  NAMES ${NITRO_CPP_NAME}
  PATHS
  /usr/lib
  /usr/local/lib)

SET(NITRO_FOUND 1)
SET(NITRO_LIBRARIES ${NITRO_CPP_LIBRARY} ${NITRO_C_LIBRARY})


IF(NITRO_INCLUDE_DIR)
  SET(NITRO_VERSION 0)

#  SET(NITRO_VERSION_H "${NITRO_INCLUDE_DIR}/nitro/c/nitf/nitf/Defines.h")
#  FILE(READ ${NITRO_VERSION_H} NITRO_VERSION_H_CONTENTS)


ENDIF()

# Handle the QUIETLY and REQUIRED arguments and set NITRO_FOUND to TRUE
# if all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Nitro DEFAULT_MSG NITRO_LIBRARIES NITRO_INCLUDE_DIR)
