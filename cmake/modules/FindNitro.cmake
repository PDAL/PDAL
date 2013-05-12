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
MESSAGE(STATUS "Searching for Nitro ${Nitro_FIND_VERSION}+ library")

IF(NITRO_INCLUDE_DIR)
  # Already in cache, be silent
  SET(NITRO_FIND_QUIETLY TRUE)
ENDIF()

FIND_PATH(NITRO_INCLUDE_DIR
  nitro
  PATHS
  /usr/include
  /usr/local/include)

SET(NITRO_C_NAME  nitf-c)

FIND_LIBRARY(NITRO_C_LIBRARY
  NAMES ${NITRO_C_NAME}
  PATHS
  /usr/lib
  /usr/local/lib)

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

  SET(NITRO_VERSION_H "${NITRO_INCLUDE_DIR}/nitro/c/nitf/Defines.h")
  FILE(READ ${NITRO_VERSION_H} NITRO_VERSION_H_CONTENTS)
  # 
  # IF (DEFINED NITRO_VERSION_H_CONTENTS)
  #   # STRING(REGEX REPLACE ".*#define[ \t]LIBGEOTIFF_VERSION[ \t]+([0-9]+).*" "\\1" GEOTIFF_VERSION_NUM "${GEOTIFF_VERSION_H_CONTENTS}")
  # 
  #   string(REGEX REPLACE "#define NITF_LIB_VERSION\w([0-9])\\.([0-9]).*" "\\1" NITRO_VERSION "${NITRO_VERSION_H_CONTENTS}")
  #   
  #   message(STATUS "NITRO_VERSION: ${NITRO_VERSION}")
  #   STRING(REGEX REPLACE "([0-9]+)\\.([0-9]+)" "\\1" NITRO_VERSION_MAJOR "${NITRO_VERSION}")
  #   STRING(REGEX REPLACE "([0-9]+)\\.([0-9]+)" "\\2" NITRO_VERSION_MINOR "${NITRO_VERSION}")
  # 
  # 
  #   SET(NITRO_VERSION "${NITRO_VERSION}"
  #     CACHE INTERNAL "The version string for Nitro library")
  # 
  #   IF (NITRO_VERSION VERSION_EQUAL Nitro_FIND_VERSION OR
  #       NITRO_VERSION VERSION_GREATER Nitro_FIND_VERSION)
  #     # MESSAGE(STATUS "Found Nitro version: ${NITRO_VERSION}")
  #   ELSE()
  #     MESSAGE(FATAL_ERROR "Nitro version check failed. Version ${NITRO_VERSION} was found, at least version ${Nitro_FIND_VERSION} is required")
  #   ENDIF()
  # ELSE()
  #   MESSAGE(FATAL_ERROR "Failed to open ${NITRO_VERSION_H} file")
  # ENDIF()

ENDIF()

# Handle the QUIETLY and REQUIRED arguments and set NITRO_FOUND to TRUE
# if all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Nitro DEFAULT_MSG NITRO_LIBRARIES NITRO_INCLUDE_DIR)
