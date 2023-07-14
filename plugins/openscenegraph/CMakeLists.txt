cmake_minimum_required(VERSION 3.5)

if (STANDALONE)
    project(pdal-openscenegraph)
    include(../../cmake/standalone.cmake NO_POLICY_SCOPE)
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

