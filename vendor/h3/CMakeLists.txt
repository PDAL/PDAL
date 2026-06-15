#
# Make sure we don't attempt to add a library more than once
#

cmake_minimum_required(VERSION 3.13)

get_property(EXISTS GLOBAL PROPERTY _PDALH3_INCLUDED)
if (EXISTS)
    return()
endif()

if (NOT DEFINED PDAL_H3_LIB_NAME)
    set(PDAL_H3_LIB_NAME pdal_h3)
endif()

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    if (DEFINED PDAL_VERSION)
        project(${PDAL_H3_LIB_NAME} VERSION ${PDAL_VERSION} LANGUAGES C CXX)
    else()
        project(${PDAL_H3_LIB_NAME} LANGUAGES C CXX)
    endif()
endif()

set(H3_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

set(H3SRCS
    "${H3_SOURCE_DIR}/lib/algos.c"
    "${H3_SOURCE_DIR}/lib/coordijk.c"
    "${H3_SOURCE_DIR}/lib/bbox.c"
    "${H3_SOURCE_DIR}/lib/polygon.c"
    "${H3_SOURCE_DIR}/lib/polyfill.c"
    "${H3_SOURCE_DIR}/lib/h3Index.c"
    "${H3_SOURCE_DIR}/lib/vec2d.c"
    "${H3_SOURCE_DIR}/lib/vec3d.c"
    "${H3_SOURCE_DIR}/lib/vertex.c"
    "${H3_SOURCE_DIR}/lib/linkedGeo.c"
    "${H3_SOURCE_DIR}/lib/localij.c"
    "${H3_SOURCE_DIR}/lib/latLng.c"
    "${H3_SOURCE_DIR}/lib/directedEdge.c"
    "${H3_SOURCE_DIR}/lib/mathExtensions.c"
    "${H3_SOURCE_DIR}/lib/iterators.c"
    "${H3_SOURCE_DIR}/lib/vertexGraph.c"
    "${H3_SOURCE_DIR}/lib/faceijk.c"
    "${H3_SOURCE_DIR}/lib/baseCells.c")

if (COMMAND PDAL_ADD_FREE_LIBRARY)
    PDAL_ADD_FREE_LIBRARY(${PDAL_H3_LIB_NAME} STATIC ${PDAL_LIB_TYPE} ${H3SRCS})
else()
    add_library(${PDAL_H3_LIB_NAME} STATIC ${H3SRCS})
endif()

target_include_directories(${PDAL_H3_LIB_NAME}
    INTERFACE
        "$<BUILD_INTERFACE:${H3_SOURCE_DIR}/include>"
        $<INSTALL_INTERFACE:include/h3>
)

target_compile_definitions(${PDAL_H3_LIB_NAME}
    PRIVATE
    H3_PREFIX=PDALH3)

target_include_directories(${PDAL_H3_LIB_NAME}
    BEFORE
    PRIVATE
        "${H3_SOURCE_DIR}/include")

if (NOT MSVC)
    target_compile_options(${PDAL_H3_LIB_NAME} PRIVATE -Wno-cast-qual)
endif()

# H3 is built static but is included in a shared lib.
set_target_properties(${PDAL_H3_LIB_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

if (DEFINED PDAL_BUILD_VERSION AND DEFINED PDAL_API_VERSION)
    set_target_properties(${PDAL_H3_LIB_NAME} PROPERTIES
        VERSION "${PDAL_BUILD_VERSION}"
        SOVERSION "${PDAL_API_VERSION}"
        CLEAN_DIRECT_OUTPUT 1)
else()
    set_target_properties(${PDAL_H3_LIB_NAME} PROPERTIES
        CLEAN_DIRECT_OUTPUT 1)
endif()

set_property(GLOBAL PROPERTY _PDALH3_INCLUDED TRUE)
