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
    LINK_WITH ${OPENTHREADS_LIBRARY} ${OSG_LIBRARY} ${OSGDB_LIBRARY})
target_include_directories(${reader_libname} PRIVATE ${OSG_INCLUDE_DIRS})

# PDAL_ADD_TEST(sqlitetest
#        FILES test/SQLiteTest.cpp
#        LINK_WITH ${reader_libname} ${writer_libname})
#    target_include_directories(sqlitetest PRIVATE
#        ${PDAL_IO_DIR}
#        ${LIBXML2_INCLUDE_DIR})
