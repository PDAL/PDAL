#
# Make sure we don't attempt to add a library more than once
#

get_property(EXISTS GLOBAL PROPERTY _PDALARBITER_INCLUDED)
if (NOT PDAL_ARBITER_ENABLED OR EXISTS)
    return()
endif()

if (CURL_FOUND)
    add_definitions("-DARBITER_CURL")
endif()

PDAL_ADD_FREE_LIBRARY(${PDAL_ARBITER_LIB_NAME} STATIC arbiter.cpp)
target_include_directories(${PDAL_ARBITER_LIB_NAME} PRIVATE
    ${PDAL_JSONCPP_INCLUDE_DIR})
target_link_libraries(${PDAL_ARBITER_LIB_NAME})
#
# Arbiter is built static but is included in a shared lib.
#
if (UNIX)
    target_compile_options(${PDAL_ARBITER_LIB_NAME} PRIVATE "-fPIC")
else()
    target_compile_definitions(${PDAL_ARBITER_LIB_NAME}
        PUBLIC -DARBITER_DLL_EXPORT -DARBITER_CURL)
endif()

set_target_properties(${PDAL_ARBITER_LIB_NAME} PROPERTIES
    VERSION "${PDAL_BUILD_VERSION}"
    SOVERSION "${PDAL_API_VERSION}"
    CLEAN_DIRECT_OUTPUT 1)

set_property(GLOBAL PROPERTY _PDALARBITER_INCLUDED TRUE)

