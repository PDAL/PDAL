###############################################################################
# CMake macro to find GDAL library.
#
# On success, the macro sets the following variables:
# GDAL_FOUND       = if the library found
# GDAL_LIBRARY     = full path to the library
# GDAL_INCLUDE_DIR = where to find the library headers 
#
# On Unix, macro sets also:
# GDAL_VERSION_STRING = human-readable string containing version of the library
#
# Author of original: Magnus Homann (Quantum GIS)
# Modifications: Mateusz Loskot <mateusz@loskot.net>
#
###############################################################################
MESSAGE(STATUS "Searching for GDAL ${GDAL_FIND_VERSION}+ library")
MESSAGE(STATUS "   NOTE: Required version is not checked - to be implemented")

SET(GDAL_NAMES gdal)

IF(WIN32)

    SET(OSGEO4W_IMPORT_LIBRARY gdal_i)
    IF(DEFINED ENV{OSGEO4W_ROOT})
        SET(OSGEO4W_ROOT_DIR $ENV{OSGEO4W_ROOT})
        MESSAGE(STATUS "Trying OSGeo4W using environment variable OSGEO4W_ROOT=$ENV{OSGEO4W_ROOT}")
    ELSE()
        SET(OSGEO4W_ROOT_DIR c:/OSGeo4W)
        MESSAGE(STATUS "Trying OSGeo4W using default location OSGEO4W_ROOT=${OSGEO4W_ROOT_DIR}")
    ENDIF()

    IF(MINGW)
        FIND_PATH(GDAL_INCLUDE_DIR
            gdal.h
            PATH_PREFIXES gdal gdal-1.6
            PATHS
            /usr/local/include
            /usr/include
            c:/msys/local/include
            ${OSGEO4W_ROOT_DIR}/include)

        FIND_LIBRARY(GDAL_LIBRARY
            NAMES ${GDAL_NAMES}
            PATH_PREFIXES gdal gdal-1.6
            PATHS
            /usr/local/lib
            /usr/lib
            c:/msys/local/lib
            ${OSGEO4W_ROOT_DIR}/lib)
    ENDIF(MINGW)

    IF(MSVC)

        FIND_PATH(GDAL_INCLUDE_DIR
            NAMES gdal.h 
            PATH_PREFIXES gdal gdal-1.6
            PATHS
            "${OSGEO4W_ROOT_DIR}/apps/gdal-dev/include"
            "$ENV{LIB_DIR}/include/gdal"
            ${OSGEO4W_ROOT_DIR}/include)

        SET(GDAL_NAMES ${OSGEO4W_IMPORT_LIBRARY} ${GDAL_NAMES})
        FIND_LIBRARY(GDAL_LIBRARY
            NAMES ${GDAL_NAMES}
            PATH_PREFIXES gdal gdal-1.6
            PATHS
            "$ENV{LIB_DIR}/lib"
            /usr/lib
            c:/msys/local/lib
            "${OSGEO4W_ROOT_DIR}/apps/gdal-dev/lib"            
            ${OSGEO4W_ROOT_DIR}/lib)
        
        IF(GDAL_LIBRARY)
            SET(GDAL_LIBRARY;odbc32;odbccp32 CACHE STRING INTERNAL)
        ENDIF()
    ENDIF(MSVC)
  
ELSEIF(UNIX)

    # Try to use framework on Mac OS X
    IF(APPLE)
        SET(GDAL_MAC_PATH /Library/Frameworks/GDAL.framework/unix/bin)
    ENDIF()

    # Try to use GDAL_HOME location if specified
    IF($ENV{GDAL_HOME})
        SET(GDAL_CONFIG_PREFER_PATH
            "$ENV{GDAL_HOME}/bin" CACHE STRING "Search for gdal-config program in preferred location")
    ENDIF()

    # Try to use OSGeo4W installation
    IF($ENV{OSGEO4W_HOME})
        SET(GDAL_CONFIG_PREFER_OSGEO4W_PATH
            "$ENV{OSGEO4W_HOME}/bin" CACHE STRING "Search for gdal-config program provided by OSGeo4W")
    ENDIF()

    # Try to use FWTools installation
    IF($ENV{FWTOOLS_HOME})
        SET(GDAL_CONFIG_PREFER_FWTOOLS_PATH
            "$ENV{FWTOOLS_HOME}/bin_safe" CACHE STRING "Search for gdal-config program provided by FWTools")
    ENDIF()

    FIND_PROGRAM(GDAL_CONFIG gdal-config
        ${GDAL_CONFIG_PREFER_PATH}
        ${GDAL_CONFIG_PREFER_OSGEO4W_PATH}
        ${GDAL_CONFIG_PREFER_FWTOOLS_PATH}
        ${GDAL_MAC_PATH}
        /usr/local/bin/
        /usr/bin/)
            
    IF(GDAL_CONFIG) 

        # TODO: Replace the regex hacks with CMake version comparison feature:
        # if(version1 VERSION_LESS version2)

        # Extract GDAL version
        EXEC_PROGRAM(${GDAL_CONFIG} ARGS --version OUTPUT_VARIABLE GDAL_VERSION)
        SET(GDAL_VERSION_STRING "${GDAL_VERSION}" CACHE STRING "Version of GDAL package found")

        STRING(REGEX REPLACE "([0-9]+)\\.([0-9]+)\\.([0-9]+)" "\\1" GDAL_VERSION_MAJOR "${GDAL_VERSION}")
        STRING(REGEX REPLACE "([0-9]+)\\.([0-9]+)\\.([0-9]+)" "\\2" GDAL_VERSION_MINOR "${GDAL_VERSION}")
  
        # Check for GDAL version
        # TODO: What version is requiredfor libLAS? --mloskot
        IF(GDAL_VERSION_MAJOR LESS 1 OR GDAL_VERSION_MINOR LESS 6)
            MESSAGE (FATAL_ERROR "GDAL version is too old (${GDAL_VERSION}). Use 1.6.0 or higher.")
        ENDIF()

        # Set INCLUDE_DIR to prefix+include
        EXEC_PROGRAM(${GDAL_CONFIG} ARGS --prefix OUTPUT_VARIABLE GDAL_PREFIX)

        FIND_PATH(GDAL_INCLUDE_DIR
            gdal.h 
            PATH_PREFIXES gdal gdal-1.6
            PATHS
            ${GDAL_PREFIX}/include/gdal
            ${GDAL_PREFIX}/include
            /usr/local/include 
            /usr/include)

        # Extract link dirs for rpath  
        EXEC_PROGRAM(${GDAL_CONFIG} ARGS --libs OUTPUT_VARIABLE GDAL_CONFIG_LIBS)

        # Split off the link dirs (for rpath)
        # Use regular expression to match wildcard equivalent "-L*<endchar>"
        # with <endchar> is a space or a semicolon
        STRING(REGEX MATCHALL "[-][L]([^ ;])+" GDAL_LINK_DIRECTORIES_WITH_PREFIX "${GDAL_CONFIG_LIBS}")
        #MESSAGE("DBG GDAL_LINK_DIRECTORIES_WITH_PREFIX=${GDAL_LINK_DIRECTORIES_WITH_PREFIX}")

        # Remove prefix -L because we need the pure directory for LINK_DIRECTORIES
        IF(GDAL_LINK_DIRECTORIES_WITH_PREFIX)
            STRING(REGEX REPLACE "[-][L]" "" GDAL_LINK_DIRECTORIES "${GDAL_LINK_DIRECTORIES_WITH_PREFIX}" )
            #MESSAGE("DBG GDAL_LINK_DIRECTORIES ${GDAL_LINK_DIRECTORIES}")
        ENDIF()

        # Split off the name
        # use regular expression to match wildcard equivalent "-l*<endchar>"
        # with <endchar> is a space or a semicolon
        STRING(REGEX MATCHALL "[-][l]([^ ;])+" GDAL_LIB_NAME_WITH_PREFIX "${GDAL_CONFIG_LIBS}")

        # Remove prefix -l because we need the pure name
        IF(GDAL_LIB_NAME_WITH_PREFIX)
            STRING(REGEX REPLACE "[-][l]" "" GDAL_LIB_NAME ${GDAL_LIB_NAME_WITH_PREFIX})
        ENDIF()

        IF(APPLE)
            SET(GDAL_LIBRARY ${GDAL_LINK_DIRECTORIES}/lib${GDAL_LIB_NAME}.dylib CACHE STRING INTERNAL)
        ELSE()
            SET(GDAL_LIBRARY ${GDAL_LINK_DIRECTORIES}/lib${GDAL_LIB_NAME}.so CACHE STRING INTERNAL)
        ENDIF()
      
    ELSE()
        MESSAGE("FindGDAL.cmake: gdal-config not found. Please set it manually: GDAL_CONFIG=${GDAL_CONFIG}")
    ENDIF(GDAL_CONFIG)

ELSE()
    MESSAGE("FindGDAL.cmake: unrecognized or unsupported operating system (use Unix or Windows)")
ENDIF()

# Handle the QUIETLY and REQUIRED arguments and set SPATIALINDEX_FOUND to TRUE
# if all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GDAL DEFAULT_MSG GDAL_LIBRARY GDAL_INCLUDE_DIR)

# TODO: Do we want to mark these as advanced? --mloskot
# http://www.cmake.org/cmake/help/cmake2.6docs.html#command:mark_as_advanced
#MARK_AS_ADVANCED(SPATIALINDEX_LIBRARY SPATIALINDEX_INCLUDE_DIR)
