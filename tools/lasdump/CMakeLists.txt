#
# The seemingly strange arrangement of this cmake file allows you to build
# lasdump as a standalone.
#
cmake_minimum_required(VERSION 2.8.6)

set(MAKE_COLOR_MAKEFILE ON)

if(NOT ROOT_DIR)
    project(LASDUMP)
    set(ROOT_DIR "${PROJECT_SOURCE_DIR}/../..")
    include(FeatureSummary)
    include(${ROOT_DIR}/cmake/common.cmake NO_POLICY_SCOPE)
    set(PDAL_UTIL_LIB_NAME pdal_util)
endif()

#
# Add necessary modules.
#
include(${PDAL_CMAKE_DIR}/laszip.cmake)

#
# Right now we don't bother if we don't have LASzip.
#
if (LASZIP_FOUND)
    add_executable(lasdump
        Dumper.cpp
        Header.cpp
    )
    target_link_libraries(lasdump PRIVATE
        ${PDAL_UTIL_LIB_NAME}
        ${LASZIP_LIBRARIES}
    )
    target_include_directories(lasdump PRIVATE
        ${PDAL_INCLUDE_DIR})
    if (WIN32)
        target_link_libraries(lasdump PRIVATE wsock32 ws2_32)
    endif()
    PDAL_TARGET_COMPILE_SETTINGS(lasdump)
endif()

