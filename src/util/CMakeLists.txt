#
# Make sure we don't attempt to add a library more than once
#
get_property(EXISTS GLOBAL PROPERTY _UTIL_INCLUDED)
if(EXISTS)
    return()
endif()

set(PDAL_UTIL_HPP
    "${PDAL_INCLUDE_DIR}/pdal/util/Algorithm.hpp"
    "${PDAL_INCLUDE_DIR}/pdal/util/Bounds.hpp"
    "${PDAL_INCLUDE_DIR}/pdal/util/Charbuf.hpp"
    "${PDAL_INCLUDE_DIR}/pdal/util/Endian.hpp"
    "${PDAL_INCLUDE_DIR}/pdal/util/Extractor.hpp"
    "${PDAL_INCLUDE_DIR}/pdal/util/FileUtils.hpp"
    "${PDAL_INCLUDE_DIR}/pdal/util/Georeference.hpp"
    "${PDAL_INCLUDE_DIR}/pdal/util/Inserter.hpp"
    "${PDAL_INCLUDE_DIR}/pdal/util/IStream.hpp"
    "${PDAL_INCLUDE_DIR}/pdal/util/OStream.hpp"
    "${PDAL_INCLUDE_DIR}/pdal/util/Utils.hpp"
    )

set(PDAL_UTIL_CPP
    "${PDAL_UTIL_DIR}/Bounds.cpp"
    "${PDAL_UTIL_DIR}/Charbuf.cpp"
    "${PDAL_UTIL_DIR}/FileUtils.cpp"
    "${PDAL_UTIL_DIR}/Georeference.cpp"
    "${PDAL_UTIL_DIR}/Utils.cpp"
    )

set(PDAL_UTIL_SOURCES
    ${PDAL_UTIL_CPP}
    ${PDAL_UTIL_HPP})

PDAL_ADD_LIBRARY(${PDAL_UTIL_LIB_NAME} SHARED ${PDAL_UTIL_SOURCES})
target_link_libraries(${PDAL_UTIL_LIB_NAME} ${Boost_LIBRARIES})

set_target_properties(${PDAL_UTIL_LIB_NAME} PROPERTIES
    VERSION "${PDAL_BUILD_VERSION}"
    SOVERSION "${PDAL_API_VERSION}"
    CLEAN_DIRECT_OUTPUT 1)

if (NOT WIN32)
    target_link_libraries(${PDAL_UTIL_LIB_NAME} dl)
endif (NOT WIN32)

if (NOT WIN32)
    target_link_libraries(${PDAL_UTIL_LIB_NAME} dl)
endif (NOT WIN32)

set_property(GLOBAL PROPERTY _UTIL_INCLUDED TRUE)

