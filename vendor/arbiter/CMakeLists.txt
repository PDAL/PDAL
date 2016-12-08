#
# Make sure we don't attempt to add a library more than once
#

if (NOT PDAL_ARBITER_ENABLED)
    return()
endif()

get_property(EXISTS GLOBAL PROPERTY _PDALARBITER_INCLUDED)
if (EXISTS)
    return()
endif()

PDAL_ADD_FREE_LIBRARY(${PDAL_ARBITER_LIB_NAME} STATIC arbiter.cpp)
target_include_directories(${PDAL_ARBITER_LIB_NAME} PRIVATE
    ${PDAL_VENDOR_DIR}/jsoncpp/dist)
target_link_libraries(${PDAL_ARBITER_LIB_NAME} )
#
# Arbiter is built static but is included in a shared lib.
#
if (UNIX)
target_compile_options(${PDAL_ARBITER_LIB_NAME} PRIVATE "-fPIC")
endif()

set_target_properties(${PDAL_ARBITER_LIB_NAME} PROPERTIES
    VERSION "${PDAL_BUILD_VERSION}"
    SOVERSION "${PDAL_API_VERSION}"
    CLEAN_DIRECT_OUTPUT 1)

set_property(GLOBAL PROPERTY _PDALARBITER_INCLUDED TRUE)

