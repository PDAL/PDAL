#
# SQLite plugin CMake configuration
#

include (${PDAL_CMAKE_DIR}/sqlite.cmake)
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
