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
)

add_executable(dimbuilder ${SOURCES})
target_link_libraries(dimbuilder PRIVATE ${PDAL_UTIL_LIB_NAME})
if (PDAL_HAVE_JSONCPP)
    target_link_libraries(dimbuilder PRIVATE ${PDAL_JSONCPP_LIB_NAME})
endif()
target_include_directories(dimbuilder PRIVATE ${PDAL_INCLUDE_DIR})
