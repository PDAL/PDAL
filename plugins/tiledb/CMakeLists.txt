#
# TileDB plugin CMake configuration
#
include(${PDAL_CMAKE_DIR}/tiledb.cmake)
if (NOT TILEDB_FOUND)
    message(FATAL_ERROR "Can't find TileDB support required.")
endif()

#
# TileDB Reader
#
PDAL_ADD_PLUGIN(reader_libname reader tiledb
    FILES
        io/TileDBReader.cpp
    LINK_WITH
        ${TILEDB_LIBRARIES}
)
target_include_directories(${reader_libname} PRIVATE
    ${PDAL_VENDOR_DIR}/pdalboost
    ${TILEDB_INCLUDE_DIR})

if (WIN32)
	target_compile_definitions(${reader_libname} PRIVATE -DNONMINMAX)
endif()

#
# TileDB Writer
#

PDAL_ADD_PLUGIN(writer_libname writer tiledb
    FILES
        io/TileDBWriter.cpp
    LINK_WITH
        ${TILEDB_LIBRARIES}
)

target_include_directories(${writer_libname} PRIVATE
    ${PDAL_VENDOR_DIR}/pdalboost
    ${TILEDB_INCLUDE_DIR})

if (WIN32)
	target_compile_definitions(${writer_libname} PRIVATE -DNONMINMAX)
endif()

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_io_tiledb_writer_test
        FILES
            test/TileDBWriterTest.cpp
        LINK_WITH
            ${writer_libname}
            ${TILEDB_LIBRARIES}
    )

    PDAL_ADD_TEST(pdal_io_tiledb_reader_test
        FILES
            test/TileDBReaderTest.cpp
        LINK_WITH
            ${reader_libname}
            ${TILEDB_LIBRARIES}
    )

if (WIN32)
	target_compile_definitions(pdal_io_tiledb_writer_test PRIVATE -DNONMINMAX)
	target_compile_definitions(pdal_io_tiledb_reader_test PRIVATE -DNONMINMAX)
endif()

endif()
