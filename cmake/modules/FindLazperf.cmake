# - Find lazperf
# Find the native LAZ-perf headers and libraries.
#
#  LAZPERF_INCLUDE_DIRS - where to find encoder.hpp.hpp, etc.
#  LAZPERF_LIBRARIES    - List of libraries when using LAZPERF.
#  LAZPERF_FOUND        - True if LAZPERF found.


# Look for the header file.
FIND_PATH(LAZPERF_INCLUDE_DIR NAMES encoder.hpp )

# Look for the library.
#FIND_LIBRARY(LAZPERF_LIBRARY NAMES msgpack libmsgpack)

# handle the QUIETLY and REQUIRED arguments and set LAZPERF_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LAZPERF DEFAULT_MSG LAZPERF_INCLUDE_DIR)

# Copy the results to the output variables.
IF(LAZPERF_FOUND)
  SET(LAZPERF_LIBRARIES ${LAZPERF_LIBRARY})
  SET(LAZPERF_INCLUDE_DIRS ${LAZPERF_INCLUDE_DIR})
ELSE(LAZPERF_FOUND)
  SET(LAZPERF_LIBRARIES)
  SET(LAZPERF_INCLUDE_DIRS)
ENDIF(LAZPERF_FOUND)

MARK_AS_ADVANCED(LAZPERF_INCLUDE_DIR LAZPERF_LIBRARY)
