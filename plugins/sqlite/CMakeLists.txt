#
# SQLite plugin CMake configuration
#

include (${PDAL_CMAKE_DIR}/sqlite.cmake)
include (${PDAL_CMAKE_DIR}/spatialite.cmake)

# SQLite Reader
#

PDAL_ADD_PLUGIN(reader_libname reader sqlite
    FILES
        io/SQLiteCommon.cpp
        io/SQLiteReader.cpp
    LINK_WITH
        ${SQLITE3_LIBRARY}
        ${LIBXML2_LIBRARIES}
    INCLUDES
        ${LIBXML2_INCLUDE_DIR}
)

#
# SQLite Writer
#

PDAL_ADD_PLUGIN(writer_libname writer sqlite
    FILES
        io/SQLiteCommon.cpp
        io/SQLiteWriter.cpp
    LINK_WITH
        ${SQLITE3_LIBRARY}
        ${GDAL_LIBRARY}
        ${LIBXML2_LIBRARIES}
    INCLUDES
        ${GDAL_INCLUDE_DIR}
        ${LIBXML2_INCLUDE_DIR}
        ${PDAL_VENDOR_DIR}/eigen
)

#
# SQLite tests
#
if(BUILD_SQLITE_TESTS)
	PDAL_ADD_TEST(sqlitetest
        FILES
            io/SQLiteCommon.cpp
            test/SQLiteTest.cpp
            test/SQLiteTest.cpp
        LINK_WITH
            ${reader_libname}
            ${writer_libname}
            ${SQLITE3_LIBRARY}
            ${LIBXML2_LIBRARY}
        INCLUDES
            ${PDAL_IO_DIR}
            ${LIBXML2_INCLUDE_DIR}
    )
endif()
