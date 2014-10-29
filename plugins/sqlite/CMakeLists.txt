#
# SQLite plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/sqlite)

find_package(SQLite3 REQUIRED)
if (SQLITE3_FOUND)
    message(STATUS "Building with SQLite support")

    mark_as_advanced(CLEAR SQLITE3_INCLUDE_DIR)
    mark_as_advanced(CLEAR SQLITE3_LIBRARY)
    include_directories(${SQLITE3_INCLUDE_DIR})

    #
    # SQLite Reader
    #
    set(srcs
	drivers/SQLiteReader.cpp
    )

    set(incs
        drivers/SQLiteCommon.hpp
	drivers/SQLiteReader.hpp
    )

    set(SQLITE_READER_LIB_NAME pdal_plugin_reader_sqlite)
    PDAL_ADD_PLUGIN(${SQLITE_READER_LIB_NAME} reader sqlite ${srcs} ${incs})
    target_link_libraries(${SQLITE_READER_LIB_NAME} ${PDAL_LINKAGE} ${SQLITE3_LIBRARY})

    #
    # SQLite Writer
    #
    set(srcs
        drivers/SQLiteWriter.cpp
    )

    set(incs
        drivers/SQLiteCommon.hpp
	drivers/SQLiteWriter.hpp
    )

    set(SQLITE_WRITER_LIB_NAME pdal_plugin_writer_sqlite)
    PDAL_ADD_PLUGIN(${SQLITE_WRITER_LIB_NAME} writer sqlite ${srcs} ${incs})
    target_link_libraries(${SQLITE_WRITER_LIB_NAME} ${PDAL_LINKAGE} ${SQLITE3_LIBRARY})

    #
    # SQLite tests
    #
    if(BUILD_SQLITE_TESTS)
	set(srcs
	    test/SqliteTest.cpp
	)

	PDAL_ADD_TEST(sqlitetest ${srcs})
    endif()
else()
    message(STATUS "Building without SQLite support")
endif()
