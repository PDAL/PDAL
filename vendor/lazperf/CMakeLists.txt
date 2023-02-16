#
# Make sure we don't attempt to add a library more than once
#

get_property(EXISTS GLOBAL PROPERTY _PDAL_LAZPERF_INCLUDED)
if (EXISTS)
    return()
endif()

file(GLOB LAZPERF_SRCS
    *.cpp
    detail/*.cpp
)

PDAL_ADD_FREE_LIBRARY(${PDAL_LAZPERF_LIB_NAME} STATIC ${LAZPERF_SRCS})
# Lazperf is built static but is included in a shared lib.
set_target_properties(${PDAL_LAZPERF_LIB_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(${PDAL_LAZPERF_LIB_NAME} PRIVATE LAZPERF_VENDORED)

set_property(GLOBAL PROPERTY _PDAL_LAZPERF_INCLUDED TRUE)
