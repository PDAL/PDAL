#
# Make sure we don't attempt to add a library more than once
#
get_property(EXISTS GLOBAL PROPERTY _UTIL_INCLUDED)
if(EXISTS)
    return()
endif()

set(PDAL_UTIL_SOURCES
    "${PDAL_UTIL_DIR}/Bounds.cpp"
    "${PDAL_UTIL_DIR}/Charbuf.cpp"
    "${PDAL_UTIL_DIR}/FileUtils.cpp"
    "${PDAL_UTIL_DIR}/Georeference.cpp"
    "${PDAL_UTIL_DIR}/Utils.cpp"
    )

PDAL_ADD_FREE_LIBRARY(${PDAL_UTIL_LIB_NAME} SHARED ${PDAL_UTIL_SOURCES})
target_link_libraries(${PDAL_UTIL_LIB_NAME}
    PRIVATE
        ${PDAL_BOOST_LIB_NAME}
)
target_include_directories(${PDAL_UTIL_LIB_NAME} PRIVATE
    ${PDAL_VENDOR_DIR}/pdalboost)

if (UNIX AND NOT APPLE)
    target_link_libraries(${PDAL_UTIL_LIB_NAME}
        PRIVATE
            dl
    )
endif()

set_target_properties(${PDAL_UTIL_LIB_NAME} PROPERTIES
    VERSION "${PDAL_BUILD_VERSION}"
    SOVERSION "${PDAL_API_VERSION}"
    CLEAN_DIRECT_OUTPUT 1)

set_property(GLOBAL PROPERTY _UTIL_INCLUDED TRUE)
