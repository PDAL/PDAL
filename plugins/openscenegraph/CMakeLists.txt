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
    INCLUDES
        ${OSG_INCLUDE_DIRS}
)

