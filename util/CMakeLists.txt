#
# Make sure we don't attempt to add a library more than once
#
get_property(EXISTS GLOBAL PROPERTY _UTIL_INCLUDED)
if(EXISTS)
    return()
endif()

set(PDAL_UTIL_HPP
    "${PDAL_UTIL_DIR}/Bounds.hpp")

set(PDAL_UTIL_CPP
    "${PDAL_UTIL_DIR}/Bounds.cpp")

set(PDAL_UTIL_SOURCES
    ${PDAL_UTIL_CPP}
    ${PDAL_UTIL_HPP})

PDAL_ADD_LIBRARY(${PDAL_UTIL_LIB_NAME} SHARED ${PDAL_UTIL_SOURCES})

install(DIRECTORY "${PDAL_UTIL_DIR}/"
  DESTINATION "${PDAL_INCLUDE_INSTALL_DIR}"
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

set_property(GLOBAL PROPERTY _UTIL_INCLUDED TRUE)

