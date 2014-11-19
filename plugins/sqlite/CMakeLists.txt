#
# SQLite plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/sqlite)
include_directories(${PROJECT_SOURCE_DIR}/drivers/las)

find_package(SQLite3)
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

    set(deps ${SQLITE3_LIBRARY})
    PDAL_ADD_PLUGIN(sqlite_reader_lib_name reader sqlite "${srcs}" "${incs}" "${deps}")

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

    set(deps ${SQLITE3_LIBRARY})
    PDAL_ADD_PLUGIN(sqlite_writer_lib_name writer sqlite "${srcs}" "${incs}" "${deps}")

    #
    # SQLite tests
    #
    if(BUILD_SQLITE_TESTS)
	set(srcs
    test/SQLiteTest.cpp
	)

        set(deps ${sqlite_reader_lib_name} ${sqlite_writer_lib_name})
	PDAL_ADD_TEST(sqlitetest "${srcs}" "${deps}")
    endif()
else()
    message(STATUS "Building without SQLite support")
endif()
