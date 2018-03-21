#
# Delaunay plugin CMake configuration
#

#include(${PDAL_CMAKE_DIR}/geogram.cmake)

PDAL_ADD_PLUGIN(filter_libname filter delaunay
    FILES
        filters/DelaunayFilter.cpp
)