#
# This file allows a test compilation of all the examples without
# a PDAL installation.
#

cmake_minimum_required(VERSION 2.8.12)
project(PdalExamples)
set(PDAL_EXAMPLES 1)
include(FeatureSummary)

set (ROOT_DIR "${PROJECT_SOURCE_DIR}/..")
include (${ROOT_DIR}/cmake/common.cmake NO_POLICY_SCOPE)
include (${ROOT_DIR}/cmake/examples.cmake)
include (${ROOT_DIR}/cmake/dimension.cmake)
include (${ROOT_DIR}/cmake/nlohmann.cmake)

add_subdirectory(../dimbuilder ./dimbuilder)

file(GLOB SRCS
    writing/*.cpp
    writing-filter/*.cpp
    writing-kernel/*.cpp
    writing-reader/*.cpp
    writing-writer/*.cpp)

add_library(dummy_lib OBJECT ${SRCS})
add_dependencies(dummy_lib generate_dimension_hpp)
target_include_directories(dummy_lib PRIVATE
    ${PDAL_INCLUDE_DIR}
    ${PROJECT_BINARY_DIR}/include)
PDAL_TARGET_COMPILE_SETTINGS(dummy_lib)

