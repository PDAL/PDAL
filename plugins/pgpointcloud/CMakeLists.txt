#
# PgPointCloud plugin CMake configuration
#

include(${PDAL_CMAKE_DIR}/postgres.cmake)

#
# PgPointCloud Reader
#
set(srcs io/PgReader.cpp)
set(incs
    io/PgCommon.hpp
    io/PgReader.hpp
)

PDAL_ADD_PLUGIN(reader_libname reader pgpointcloud
    FILES "${srcs}" "${incs}"
    LINK_WITH ${POSTGRESQL_LIBRARIES})

#
# PgPointCloud Writer
#
set(srcs io/PgWriter.cpp)
set(incs
    io/PgCommon.hpp
    io/PgWriter.hpp
)

PDAL_ADD_PLUGIN(writer_libname writer pgpointcloud
    FILES "${srcs}" "${incs}"
    LINK_WITH ${POSTGRESQL_LIBRARIES})

#
# PgPointCloud tests
#
if (BUILD_PGPOINTCLOUD_TESTS)
	set(PGPOINTCLOUD_TEST_DB_USER $ENV{PGUSER} CACHE STRING
            "Postgres test database user, must be able to create databases")
	set(PGPOINTCLOUD_TEST_DB_HOST $ENV{PGHOST} CACHE STRING
            "Postgres test database host")
	set(PGPOINTCLOUD_TEST_DB_PORT $ENV{PGPORT} CACHE STRING
            "Postgres test database port")
	set(PGPOINTCLOUD_TEST_DB_NAME template1 CACHE STRING
            "Postgres master database name, must exist")
	set(PGPOINTCLOUD_TEST_DB_TEMPNAME pdal_test_tmp CACHE STRING
            "Postgres test database temp database name")

	configure_file(
        test/Pgtest-Support.hpp.in
        ${CMAKE_CURRENT_BINARY_DIR}/Pgtest-Support.hpp
    )

    set(srcs
        test/PgpointcloudWriterTest.cpp
        ${CMAKE_CURRENT_BINARY_DIR}/Pgtest-Support.hpp
    )

    include_directories(${CMAKE_CURRENT_BINARY_DIR})

	PDAL_ADD_TEST(pgpointcloudtest
        FILES "${srcs}"
        LINK_WITH ${reader_libname} ${writer_libname})
endif()
