#
# The seemingly strange arrangement of this cmake file allows you to build
# lasdump as a standalone.
#
cmake_minimum_required(VERSION 2.8.6)

if(NOT ROOT_DIR)
    project(NITFWRAP)
    set(ROOT_DIR "${PROJECT_SOURCE_DIR}/../..")
endif()
include(${ROOT_DIR}/cmake/common.cmake NO_POLICY_SCOPE)
if (PROJECT_NAME STREQUAL "NITFWRAP")
    include(${PDAL_CMAKE_DIR}/dimension.cmake)
endif()
include(${PDAL_CMAKE_DIR}/nitro.cmake)

set(pdal_defines_h_in "${CMAKE_CURRENT_SOURCE_DIR}/pdal_defines.h.in")
set(pdal_defines_h "${CMAKE_CURRENT_BINARY_DIR}/include/pdal/pdal_defines.h")
configure_file(${pdal_defines_h_in} ${pdal_defines_h})

set(PDAL_NITF_DIR ${ROOT_DIR}/plugins/nitf/io)

include_directories("${CMAKE_CURRENT_BINARY_DIR}/include")
include_directories(${PDAL_INCLUDE_DIR} ${PDAL_NITF_DIR})

#
# Right now we don't bother if we don't have LASzip.
#
set (SOURCES
    NitfWrap.cpp
    ${PDAL_NITF_DIR}/NitfFileWriter.cpp
    ${PDAL_NITF_DIR}/tre_plugins.cpp
)

set (HEADERS
    NitfWrap.hpp
    ${PDAL_NITF_DIR}/NitfFileWriter.hpp
    ${PDAL_NITF_DIR}/tre_plugins.hpp
)

add_executable(nitfwrap ${SOURCES} ${HEADERS})
add_dependencies(nitfwrap generate_dimension_hpp)
target_link_libraries(nitfwrap
    ${PDAL_BASE_LIB_NAME}
    ${PDAL_UTIL_LIB_NAME}
    ${NITRO_LIBRARIES}
)

