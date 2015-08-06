# Finds Matlab's libmat, and nothing more
#
# This is a custom script for PDAL. CMake's provided FindMatlab doesn't
# find libmat.
#
# Defines:
#
#  MATLAB_INCLUDE_DIR: the Matlab include path
#  MATLAB_LIBRARIES: path to Matlab libraries
#
# Set MATLAB_ROOT to help cmake find the right stuff.

set(MATLAB_FOUND 0)


set(MATLAB_EXTERN_DIR ${MATLAB_ROOT}/extern/include)
if (APPLE)
    set(MATLAB_LIB_DIR ${MATLAB_ROOT}/bin/maci64)
elseif (UNIX)
    set(MATLAB_LIB_DIR ${MATLAB_ROOT}/bin/glnxa64)
endif()

message(STATUS ${MATLAB_LIB_DIR})

find_library(MATLAB_MAT_LIBRARY
    mat
    ${MATLAB_LIB_DIR}
    )
find_library(MATLAB_MX_LIBRARY
    mx
    ${MATLAB_LIB_DIR}
    )
find_path(MATHALB_INCLUDE_DIR
    "mat.h"
    ${MATLAB_EXTERN_DIR}
    )

set(MATLAB_LIBRARIES
    ${MATLAB_MAT_LIBRARY}
    ${MATLAB_MX_LIBRARY}
    )
mark_as_advanced(MATLAB_LIBRARIES)


if (MATLAB_MAT_LIBRARIES AND MATLAB_INCLUDE_DIR)
    set(MATLAB_FOUND 1)
endif()
