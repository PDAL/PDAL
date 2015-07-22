###############################################################################
#
# CMake module to search for Jace library
#
# On success, the macro sets the following variables:
# JACE_FOUND       = if the library found
# JACE_LIBRARIES   = full path to the library
# JACE_INCLUDE_DIR = where to find the library headers also defined,
#                       but not for general use are
# JACE_LIBRARY     = where to find the Jace library.
#
###############################################################################
MESSAGE(STATUS "Searching for Jace library")

IF(JACE_INCLUDE_DIR)
  # Already in cache, be silent
  SET(JACE_FIND_QUIETLY TRUE)
ENDIF()

FIND_PATH(JACE_INCLUDE_DIR
  Jace.h
  PATHS
  /usr/include
  /usr/local
  /usr/local/include)

SET(JACE_NAMES libjace.so jace.lib)

FIND_LIBRARY(JACE_LIBRARY
  NAMES ${JACE_NAMES}
  PATHS
  /usr/lib
  /usr/local
  /usr/local/lib)

FIND_FILE(JACE_RUNTIME_JAR
  jace-runtime.jar
  PATHS
  /usr/bin
  /usr/local
  /usr/local/bin)

# Handle the QUIETLY and REQUIRED arguments and set JACE_FOUND to TRUE
# if all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Jace DEFAULT_MSG JACE_RUNTIME_JAR JACE_LIBRARY JACE_INCLUDE_DIR)
