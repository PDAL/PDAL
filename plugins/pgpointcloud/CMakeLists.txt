#
# PgPointCloud plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/pgpointcloud)

find_package(PostgreSQL)
if (POSTGRESQL_FOUND)
    message(STATUS "Building with PgPointCloud support")

    mark_as_advanced(CLEAR POSTGRESQL_INCLUDE_DIR)
    mark_as_advanced(CLEAR POSTGRESQL_LIBRARIES)
    include_directories(${POSTGRESQL_INCLUDE_DIR})

    #
    # PgPointCloud Reader
    #
    set(srcs
	drivers/PgReader.cpp
    )

    set(incs
	drivers/PgCommon.hpp
	drivers/PgReader.hpp
    )

    set(deps ${POSTGRESQL_LIBRARIES})
    PDAL_ADD_PLUGIN(pgpointcloud_reader_lib_name reader pgpointcloud "${srcs}" "${incs}" "${deps}")

    #
    # PgPointCloud Writer
    #
    set(srcs
	drivers/PgWriter.cpp
    )

    set(incs
	drivers/PgCommon.hpp
	drivers/PgWriter.hpp
    )

    set(deps ${POSTGRESQL_LIBRARIES})
    PDAL_ADD_PLUGIN(pgpointcloud_writer_lib_name writer pgpointcloud "${srcs}" "${incs}" "${deps}")

    #
    # PgPointCloud tests
    #
    if (BUILD_PGPOINTCLOUD_TESTS)
	set(PGPOINTCLOUD_TEST_DB_HOST localhost CACHE STRING "Postgres test database host")
	set(PGPOINTCLOUD_TEST_DB_PORT 5432 CACHE STRING "Postgres test database port")
	set(PGPOINTCLOUD_TEST_DB_NAME pdal_test CACHE STRING
	    "Postgres test database name, must exist and must be able to create databases")
	set(PGPOINTCLOUD_TEST_DB_TEMPNAME pdal_test_tmp CACHE STRING "Postgres test database temp database name")

	configure_file(
	    test/Pgtest-Support.hpp.in
	    ${CMAKE_CURRENT_BINARY_DIR}/Pgtest-Support.hpp
        )

        set(srcs
	    test/PgpointcloudWriterTest.cpp
	    ${CMAKE_CURRENT_BINARY_DIR}/Pgtest-Support.hpp
	)

        PDAL_ADD_TEST(pgpointcloudtest "${srcs}")
    endif()
else()
    message(STATUS "Building without PgPointCloud support")
endif()
