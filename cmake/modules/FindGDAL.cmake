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

# Computes the realtionship between two version strings.  A version
# string is a number delineated by '.'s such as 1.3.2 and 0.99.9.1.
# You can feed version strings with different number of dot versions,
# and the shorter version number will be padded with zeros: 9.2 <
# 9.2.1 will actually compare 9.2.0 < 9.2.1.
#
# Input: a_in - value, not variable
#        b_in - value, not variable
#        result_out - variable with value:
#                         -1 : a_in <  b_in
#                          0 : a_in == b_in
#                          1 : a_in >  b_in
#
# Written by James Bigler.
MACRO(COMPARE_VERSION_STRINGS a_in b_in result_out)
  # Since SEPARATE_ARGUMENTS using ' ' as the separation token,
  # replace '.' with ' ' to allow easy tokenization of the string.
  STRING(REPLACE "." " " a ${a_in})
  STRING(REPLACE "." " " b ${b_in})
  SEPARATE_ARGUMENTS(a)
  SEPARATE_ARGUMENTS(b)

  # Check the size of each list to see if they are equal.
  LIST(LENGTH a a_length)
  LIST(LENGTH b b_length)

  # Pad the shorter list with zeros.

  # Note that range needs to be one less than the length as the for
  # loop is inclusive (silly CMake).
  IF(a_length LESS b_length)
    # a is shorter
    SET(shorter a)
    MATH(EXPR range "${b_length} - 1")
    MATH(EXPR pad_range "${b_length} - ${a_length} - 1")
  ELSE(a_length LESS b_length)
    # b is shorter
    SET(shorter b)
    MATH(EXPR range "${a_length} - 1")
    MATH(EXPR pad_range "${a_length} - ${b_length} - 1")
  ENDIF(a_length LESS b_length)

  # PAD out if we need to
  IF(NOT pad_range LESS 0)
    FOREACH(pad RANGE ${pad_range})
      # Since shorter is an alias for b, we need to get to it by by dereferencing shorter.
      LIST(APPEND ${shorter} 0)
    ENDFOREACH(pad RANGE ${pad_range})
  ENDIF(NOT pad_range LESS 0)

  SET(result 0)
  FOREACH(index RANGE ${range})
    IF(result EQUAL 0)
      # Only continue to compare things as long as they are equal
      LIST(GET a ${index} a_version)
      LIST(GET b ${index} b_version)
      # LESS
      IF(a_version LESS b_version)
        SET(result -1)
      ENDIF(a_version LESS b_version)
      # GREATER
      IF(a_version GREATER b_version)
        SET(result 1)
      ENDIF(a_version GREATER b_version)
    ENDIF(result EQUAL 0)
  ENDFOREACH(index)

  # Copy out the return result
  SET(${result_out} ${result})
ENDMACRO(COMPARE_VERSION_STRINGS)

MESSAGE(STATUS "Searching for GDAL ${GDAL_FIND_VERSION}+ library")
set (GDAL_VERSION_COUNT 3)

SET(GDAL_NAMES gdal)

IF(WIN32)

    SET(OSGEO4W_IMPORT_LIBRARY gdal_i)
    IF(DEFINED ENV{OSGEO4W_ROOT})
        SET(OSGEO4W_ROOT_DIR $ENV{OSGEO4W_ROOT})
        #MESSAGE(STATUS " FindGDAL: trying OSGeo4W using environment variable OSGEO4W_ROOT=$ENV{OSGEO4W_ROOT}")
    ELSE()
        SET(OSGEO4W_ROOT_DIR c:/OSGeo4W)
        #MESSAGE(STATUS " FindGDAL: trying OSGeo4W using default location OSGEO4W_ROOT=${OSGEO4W_ROOT_DIR}")
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
        EXEC_PROGRAM(${GDAL_CONFIG} ARGS --version OUTPUT_VARIABLE GDAL_VERSION_STATED)
        SET(GDAL_VERSION_STRING "${GDAL_VERSION_STATED}" CACHE STRING "Version of GDAL package found")

        STRING(REGEX REPLACE "([0-9]+)\\.([0-9]+)\\.([0-9]+)" "\\1" GDAL_VERSION_MAJOR "${GDAL_VERSION_STATED}")
        STRING(REGEX REPLACE "([0-9]+)\\.(/^\\d{1,2}$/)\\.([0-9]+)" "\\2" GDAL_VERSION_MINOR "${GDAL_VERSION_STATED}")
        STRING(REGEX REPLACE "([0-9]+)\\.([0-9]+)\\.([0-9]+)" "\\3" GDAL_VERSION_PATCH "${GDAL_VERSION_STATED}")
        
        # Check for GDAL version
        if (GDAL_FIND_VERSION)
        COMPARE_VERSION_STRINGS( "${GDAL_VERSION_STATED}" "${GDAL_FIND_VERSION}" version_result)
        IF (version_result LESS 0)
            MESSAGE (FATAL_ERROR "GDAL version is too old (${GDAL_VERSION_STATED}). Use ${GDAL_FIND_VERSION} or higher requested.")
        else()
            MESSAGE(STATUS "Found acceptable GDAL version ${GDAL_VERSION_STATED}")
            set (GDAL_VERSION_STRING ${GDAL_VERSION_STATED})
            # set (GDAL_VERSION "1.6")
            set (GDAL_VERSION_COMPATIBLE true)
        ENDIF()
        endif()

        set (GDAL_FOUND TRUE)
        # Set INCLUDE_DIR to prefix+include
        EXEC_PROGRAM(${GDAL_CONFIG} ARGS --prefix OUTPUT_VARIABLE GDAL_PREFIX)

        FIND_PATH(GDAL_INCLUDE_DIR
            gdal.h 
            PATH_PREFIXES gdal
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

# Handle the QUIETLY and REQUIRED arguments and set GDAL_FOUND to TRUE
# if all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GDAL DEFAULT_MSG GDAL_LIBRARY GDAL_INCLUDE_DIR)

