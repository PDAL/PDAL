#
# Make sure we don't attempt to add a library more than once
#

get_property(EXISTS GLOBAL PROPERTY _PDALH3_INCLUDED)
if (EXISTS)
    return()
endif()


set(H3SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/alloc.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/bbox.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/polygon.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/polygonAlgos.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/polyfill.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/h3Index.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/directedEdge.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/latLng.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/vec2d.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/vec3d.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/linkedGeo.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/localij.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/baseCells.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/faceijk.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/vertex.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/vertexGraph.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/mathExtensions.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/iterators.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/constants.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/coordijk.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/algos.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/algos.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/coordijk.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/bbox.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/polygon.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/polyfill.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/h3Index.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/vec2d.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/vec3d.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/vertex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/linkedGeo.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/localij.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/latLng.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/directedEdge.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/mathExtensions.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/iterators.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/vertexGraph.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/faceijk.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/baseCells.c")


PDAL_ADD_FREE_LIBRARY(${PDAL_H3_LIB_NAME} STATIC ${H3SRCS})

target_compile_definitions(${PDAL_H3_LIB_NAME}
    PRIVATE
    H3_PREFIX=PDALH3)

target_include_directories(${PDAL_H3_LIB_NAME}
    BEFORE
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include")

if (NOT MSVC)
    target_compile_options(${PDAL_H3_LIB_NAME} PRIVATE -Wno-cast-qual)
endif()


# H3 is built static but is included in a shared lib.
set_target_properties(${PDAL_H3_LIB_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)


set_target_properties(${PDAL_H3_LIB_NAME} PROPERTIES
    VERSION "${PDAL_BUILD_VERSION}"
    SOVERSION "${PDAL_API_VERSION}"
    CLEAN_DIRECT_OUTPUT 1)

set_property(GLOBAL PROPERTY _PDALH3_INCLUDED TRUE)

