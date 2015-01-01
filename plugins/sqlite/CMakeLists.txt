#
# SQLite plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/sqlite)
include_directories(${PROJECT_SOURCE_DIR}/io/las)

find_package(SQLite3)
if (SQLITE3_FOUND)
    message(STATUS "Building with SQLite support")

    mark_as_advanced(CLEAR SQLITE3_INCLUDE_DIR)
    mark_as_advanced(CLEAR SQLITE3_LIBRARY)
    include_directories(${SQLITE3_INCLUDE_DIR})

    #
    # SQLite Reader
    #
    set(srcs io/SQLiteReader.cpp)
    set(incs
        io/SQLiteCommon.hpp
        io/SQLiteReader.hpp
    )

    PDAL_ADD_PLUGIN(reader_libname reader sqlite
        FILES "${srcs}" "${incs}"
        LINK_WITH ${SQLITE3_LIBRARY})

    #
    # SQLite Writer
    #
    set(srcs io/SQLiteWriter.cpp)
    set(incs
        io/SQLiteCommon.hpp
        io/SQLiteWriter.hpp
    )

    PDAL_ADD_PLUGIN(writer_libname writer sqlite
        FILES "${srcs}" "${incs}"
        LINK_WITH ${SQLITE3_LIBRARY})

    #
    # SQLite tests
    #
    if(BUILD_SQLITE_TESTS)
	PDAL_ADD_TEST(sqlitetest
            FILES test/SQLiteTest.cpp
            LINK_WITH ${reader_libname} ${writer_libname})
    endif()
else()
    message(STATUS "Building without SQLite support")
endif()
