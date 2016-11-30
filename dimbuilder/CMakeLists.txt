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

#
# We include Utils.cpp here rather than linking in the Utils library because
# some people building linux packages disable rpath, which means that
# the Utils library can't be located when running dimbuilder, even though
# it has been built.
#
set (SOURCES
    DimBuilder.cpp
    ${PDAL_JSONCPP_SRC}
    ${PDAL_UTIL_DIR}/Utils.cpp
)

add_executable(dimbuilder ${SOURCES})
if (PDAL_HAVE_JSONCPP)
    target_link_libraries(dimbuilder PRIVATE ${PDAL_JSONCPP_LIB_NAME})
endif()
target_include_directories(dimbuilder PRIVATE ${PDAL_INCLUDE_DIR})
