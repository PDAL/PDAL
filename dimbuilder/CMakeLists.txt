#
# The seemingly strange arrangement of this cmake file allows you to build
# lasdump as a standalone.
#
cmake_minimum_required(VERSION 2.8.6)

set(MAKE_COLOR_MAKEFILE ON)

if(NOT ROOT_DIR)
    project(DIMBUILDER)
    set(ROOT_DIR "${PROJECT_SOURCE_DIR}/..")
endif()

include(${ROOT_DIR}/cmake/common.cmake NO_POLICY_SCOPE)

include_directories(${PDAL_JSONCPP_INCLUDE_DIR})


set (SOURCES
    DimBuilder.cpp
    ${PDAL_JSONCPP_SRC}
    ${PDAL_UTIL_DIR}/Utils.cpp
)

set (HEADERS
    DimBuilder.hpp
    ${PDAL_INCLUDE_DIR}/pdal/util/Utils.hpp
)

add_executable(dimbuilder ${SOURCES} ${HEADERS})
if (PDAL_HAVE_JSONCPP)
    target_link_libraries(dimbuilder ${PDAL_JSONCPP_LIB_NAME})
endif()
if (UNIX AND NOT APPLE)
    target_link_libraries(dimbuilder PUBLIC dl)
endif()
