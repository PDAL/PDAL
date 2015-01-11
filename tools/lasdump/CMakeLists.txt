#
# The seemingly strange arrangement of this cmake file allows you to build
# lasdump as a standalone.
#
cmake_minimum_required(VERSION 2.8.6)

set(MAKE_COLOR_MAKEFILE ON)

if(NOT ROOT_DIR)
    project(LASDUMP)
    set(ROOT_DIR "${PROJECT_SOURCE_DIR}/../..")
endif()
include(${ROOT_DIR}/cmake/common.cmake NO_POLICY_SCOPE)

include_directories(${PDAL_UTIL_DIR} ${PDAL_INCLUDE_DIR})

#
# Add necessary modules.
#
include(${PDAL_CMAKE_DIR}/laszip.cmake)
find_package(LASzip QUIET)

#
# Right now we don't bother if we don't have LASzip.
#
if (LASZIP_FOUND)
    set (SOURCES
        Dumper.cpp
        Header.cpp
    )

    set (HEADERS
        Dumper.hpp
        Header.hpp
        Lasdump.hpp
        Vlr.hpp
    )

    add_executable(lasdump ${SOURCES} ${HEADERS})
    target_link_libraries(lasdump
        ${PDAL_UTIL_LIB_NAME}
        ${LASZIP_LIBRARIES}
    )
endif()

