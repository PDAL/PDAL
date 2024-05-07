##############################################################################
#
# CMake module to search for the TileDB library
#
# On success, the macro sets the following variables:
# TileDB_FOUND       = if the library found
# TileDB_LIBRARIES   = full path to the library
# TileDB_INCLUDE_DIR = where to find the library headers also defined,
#                       but not for general use are
# TileDB_LIBRARY     = where to find the hexer library.
# TileDB_VERSION     = version of library which was found, e.g. "1.4.1"
#
# Copyright (c) 2019 TileDB, Inc
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#
###############################################################################

IF(TileDB_INCLUDE_DIR)
    # Already in cache, be silent
    SET(TileDB_FIND_QUIETLY TRUE)
ENDIF()

# Note: the environment variable is all caps unlike the find_module components.
IF(DEFINED ENV{TILEDB_HOME})
    set(TileDB_HOME $ENV{TILEDB_HOME})
ENDIF()

FIND_PATH(TileDB_INCLUDE_DIR
        tiledb
        PATHS
        /usr/include
        /usr/local/include
        ${TileDB_HOME}/dist/include)

FIND_LIBRARY(TileDB_LIBRARY
        NAMES tiledb
        PATHS
        /usr/lib
        /usr/local/lib
        ${TileDB_HOME}/dist/lib)

SET(TileDB_VERSION_H "${TileDB_INCLUDE_DIR}/tiledb/tiledb_version.h")
IF(TileDB_INCLUDE_DIR AND EXISTS ${TileDB_VERSION_H})
    SET(TileDB_VERSION 0)

    FILE(READ ${TileDB_VERSION_H} TileDB_VERSION_H_CONTENTS)

    IF (DEFINED TileDB_VERSION_H_CONTENTS)
        string(REGEX REPLACE ".*#define TILEDB_VERSION_MAJOR ([0-9]+).*" "\\1" TileDB_VERSION_MAJOR "${TileDB_VERSION_H_CONTENTS}")
        string(REGEX REPLACE ".*#define TILEDB_VERSION_MINOR ([0-9]+).*" "\\1" TileDB_VERSION_MINOR "${TileDB_VERSION_H_CONTENTS}")
        string(REGEX REPLACE ".*#define TILEDB_VERSION_PATCH ([0-9]+).*" "\\1" TileDB_VERSION_PATCH "${TileDB_VERSION_H_CONTENTS}")

        if(NOT "${TileDB_VERSION_MAJOR}" MATCHES "^[0-9]+$")
            message(FATAL_ERROR "TileDB version parsing failed for \"TILEDB_API_VERSION_MAJOR\"")
        endif()
        if(NOT "${TileDB_VERSION_MINOR}" MATCHES "^[0-9]+$")
            message(FATAL_ERROR "TileDB version parsing failed for \"TILEDB_VERSION_MINOR\"")
        endif()
        if(NOT "${TileDB_VERSION_PATCH}" MATCHES "^[0-9]+$")
            message(FATAL_ERROR "TileDB version parsing failed for \"TILEDB_VERSION_PATCH\"")
        endif()

        SET(TileDB_VERSION "${TileDB_VERSION_MAJOR}.${TileDB_VERSION_MINOR}.${TileDB_VERSION_PATCH}"
                CACHE INTERNAL "The version string for TileDB library")

        IF (TileDB_VERSION VERSION_LESS TileDB_FIND_VERSION)
            MESSAGE(FATAL_ERROR "TileDB version check failed. Version ${TileDB_VERSION} was found, at least version ${TileDB_FIND_VERSION} is required")
        ENDIF()
    ELSE()
        MESSAGE(FATAL_ERROR "Failed to open ${TileDB_VERSION_H} file")
    ENDIF()

ENDIF()

SET(TileDB_LIBRARIES ${TileDB_LIBRARY})

# Handle the QUIETLY and REQUIRED arguments and set TileDB_FOUND to TRUE
# if all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
    TileDB
    REQUIRED_VARS TileDB_LIBRARY TileDB_INCLUDE_DIR
    VERSION_VAR TileDB_VERSION
    HANDLE_VERSION_RANGE
)
