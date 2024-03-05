#
# Make sure we don't attempt to add a library more than once
#

get_property(EXISTS GLOBAL PROPERTY _PDALH3_INCLUDED)
if (EXISTS)
    return()
endif()


set(H3SRCS
    ./include/h3Assert.h
    ./include/alloc.h
    ./include/bbox.h
    ./include/polygon.h
    ./include/polygonAlgos.h
    ./include/polyfill.h
    ./include/h3Index.h
    ./include/directedEdge.h
    ./include/latLng.h
    ./include/vec2d.h
    ./include/vec3d.h
    ./include/linkedGeo.h
    ./include/localij.h
    ./include/baseCells.h
    ./include/faceijk.h
    ./include/vertex.h
    ./include/vertexGraph.h
    ./include/mathExtensions.h
    ./include/iterators.h
    ./include/constants.h
    ./include/coordijk.h
    ./include/algos.h
    ./lib/h3Assert.c
    ./lib/algos.c
    ./lib/coordijk.c
    ./lib/bbox.c
    ./lib/polygon.c
    ./lib/polyfill.c
    ./lib/h3Index.c
    ./lib/vec2d.c
    ./lib/vec3d.c
    ./lib/vertex.c
    ./lib/linkedGeo.c
    ./lib/localij.c
    ./lib/latLng.c
    ./lib/directedEdge.c
    ./lib/mathExtensions.c
    ./lib/iterators.c
    ./lib/vertexGraph.c
    ./lib/faceijk.c
    ./lib/baseCells.c)


PDAL_ADD_FREE_LIBRARY(${PDAL_H3_LIB_NAME} STATIC ${H3SRCS})

target_compile_definitions(${PDAL_H3_LIB_NAME}
    PRIVATE
    H3_PREFIX=PDALH3)

target_include_directories(${PDAL_H3_LIB_NAME}
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include")



# H3 is built static but is included in a shared lib.
set_target_properties(${PDAL_H3_LIB_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)



set_target_properties(${PDAL_H3_LIB_NAME} PROPERTIES
    VERSION "${PDAL_BUILD_VERSION}"
    SOVERSION "${PDAL_API_VERSION}"
    CLEAN_DIRECT_OUTPUT 1)

set_property(GLOBAL PROPERTY _PDALH3_INCLUDED TRUE)

