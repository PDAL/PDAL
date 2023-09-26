cmake_minimum_required(VERSION 3.13)

if (STANDALONE)
    project(pdal-openscenegraph)
    include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/standalone.cmake" )
    find_package(PDAL REQUIRED)
endif(STANDALONE)

#
# OpenScenGraph plugin CMake configuration
#

include (${PDAL_CMAKE_DIR}/openscenegraph.cmake)

# OpenSceneGraph Reader
#

PDAL_ADD_PLUGIN(reader_libname reader openscenegraph
    FILES
        io/OSGReader.cpp
        io/OSGReader.hpp
    LINK_WITH
        ${OPENTHREADS_LIBRARY}
        ${OSG_LIBRARY}
        ${OSGDB_LIBRARY}
        ${PDAL_LIBRARIES}
    INCLUDES
        ${OSG_INCLUDE_DIRS}
)

