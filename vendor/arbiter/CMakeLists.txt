#
# Make sure we don't attempt to add a library more than once
#
get_property(EXISTS GLOBAL PROPERTY _PDALARBITER_INCLUDED)
if (EXISTS)
    return()
endif()

file(GLOB PDAL_ARBITER_SOURCES
    "arbiter.cpp"
    "arbiter.hpp"
)

PDAL_ADD_LIBRARY(${PDAL_ARBITER_LIB_NAME} STATIC "${PDAL_ARBITER_SOURCES}")
target_link_libraries(${PDAL_ARBITER_LIB_NAME} )

set_target_properties(${PDAL_ARBITER_LIB_NAME} PROPERTIES
    VERSION "${PDAL_BUILD_VERSION}"
    SOVERSION "${PDAL_API_VERSION}"
    CLEAN_DIRECT_OUTPUT 1)

set_property(GLOBAL PROPERTY _PDALARBITER_INCLUDED TRUE)

