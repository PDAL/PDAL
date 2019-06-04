#
# The seemingly strange arrangement of this cmake file allows you to build
# this application as a standalone.
#
cmake_minimum_required(VERSION 2.8.6)

set(MAKE_COLOR_MAKEFILE ON)

if(NOT ROOT_DIR)
    project(DIMBUILDER)
    set(ROOT_DIR "${PROJECT_SOURCE_DIR}/..")
endif()

include(${ROOT_DIR}/cmake/common.cmake NO_POLICY_SCOPE)

#
# We include Utils.cpp here rather than linking in the Utils library because
# some people building linux packages disable rpath, which means that
# the Utils library can't be located when running dimbuilder, even though
# it has been built.
#
add_executable(dimbuilder
    DimBuilder.cpp
    ${PDAL_UTIL_DIR}/Utils.cpp
)
target_include_directories(dimbuilder PRIVATE
    ${PDAL_INCLUDE_DIR}
    ${NLOHMANN_INCLUDE_DIR}
)
PDAL_TARGET_COMPILE_SETTINGS(dimbuilder)
target_link_libraries(dimbuilder
    PRIVATE
        ${EXECINFO_LIBRARY}
        ${CMAKE_DL_LIBS}
)
