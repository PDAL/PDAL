cmake_minimum_required(VERSION 3.5)

if (STANDALONE)
    project(pdal-tiledb)
    include("${CMAKE_SOURCE_DIR}/../../cmake/standalone.cmake" )
    include(${PDAL_CMAKE_DIR}/nlohmann.cmake)
endif(STANDALONE)

#
# TileDB plugin CMake configuration
#
include(${PDAL_CMAKE_DIR}/tiledb.cmake)
if (NOT TileDB_FOUND)
    message(FATAL_ERROR "Can't find TileDB support required.")
endif()

if (WIN32)
    add_definitions(-DNOMINMAX)
endif()

#
# TileDB Reader
#
PDAL_ADD_PLUGIN(reader_libname reader tiledb
    FILES
        io/TileDBReader.cpp
        io/TileDBUtils.cpp
    LINK_WITH
        ${TileDB_LIBRARIES}
        ${PDAL_LIBRARIES}
    INCLUDES
        ${TileDB_INCLUDE_DIR}
        ${NLOHMANN_INCLUDE_DIR}
)

#
# TileDB Writer
#
PDAL_ADD_PLUGIN(writer_libname writer tiledb
    FILES
        io/TileDBWriter.cpp
        io/TileDBUtils.cpp
    LINK_WITH
        ${TileDB_LIBRARIES}
        ${PDAL_LIBRARIES}
    INCLUDES
        ${TileDB_INCLUDE_DIR}
        ${NLOHMANN_INCLUDE_DIR}
)

if (WITH_TESTS)

    PDAL_ADD_TEST(pdal_io_tiledb_utils_test
        FILES
            test/TileDBUtilsTest.cpp
            io/TileDBUtils.cpp
        LINK_WITH
            ${TileDB_LIBRARIES}
        INCLUDES
            ${TileDB_INCLUDE_DIR}
            ${NLOHMANN_INCLUDE_DIR}
    )

    PDAL_ADD_TEST(pdal_io_tiledb_writer_test
        FILES
            test/TileDBWriterTest.cpp
        LINK_WITH
            ${writer_libname}
            ${TileDB_LIBRARIES}
        INCLUDES
            ${TileDB_INCLUDE_DIR}
            ${NLOHMANN_INCLUDE_DIR}
    )

    PDAL_ADD_TEST(pdal_io_tiledb_reader_test
        FILES
            test/TileDBReaderTest.cpp
        LINK_WITH
            ${reader_libname}
            ${writer_libname}
            ${TileDB_LIBRARIES}
        INCLUDES
            ${TileDB_INCLUDE_DIR}
            ${NLOHMANN_INCLUDE_DIR}
    )

    PDAL_ADD_TEST(pdal_io_tiledb_time_writer_test
        FILES
            test/TileDBWriterTimeDimTest.cpp
            test/XYZTmUtils.cpp
        LINK_WITH
            ${reader_libname}
            ${writer_libname}
            ${TileDB_LIBRARIES}
        INCLUDES
            ${TileDB_INCLUDE_DIR}
            ${NLOHMANN_INCLUDE_DIR}
    )

    PDAL_ADD_TEST(pdal_io_tiledb_time_reader_test
        FILES
            test/TileDBReaderTimeDimTest.cpp
            test/XYZTmUtils.cpp
        LINK_WITH
            ${reader_libname}
            ${writer_libname}
            ${TileDB_LIBRARIES}
        INCLUDES
            ${TileDB_INCLUDE_DIR}
            ${NLOHMANN_INCLUDE_DIR}
    )


endif()
