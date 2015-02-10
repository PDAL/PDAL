###############################################################################
#
# CMake module to search for GeoWave
#
# On success, the macro sets the following variables:
# GeoWave_FOUND    = if the runtime jar is found
# GeoWave_RUNTIME  = full path to the GeoWave jar
#
###############################################################################
MESSAGE(STATUS "Searching for GeoWave")

IF(GEOWAVE_RUNTIME_JAR)
  # Already in cache, be silent
  SET(GEOWAVE_FIND_QUIETLY TRUE)
ENDIF()

FIND_FILE(GEOWAVE_RUNTIME_JAR
  geowave-deploy-${GeoWave_FIND_VERSION}-accumulo-singlejar.jar
  PATHS
  /usr/bin
  /usr/local
  /usr/local/bin)

# Handle the QUIETLY and REQUIRED arguments and set GeoWave_FOUND to TRUE
# if all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GeoWave DEFAULT_MSG GEOWAVE_RUNTIME_JAR)
