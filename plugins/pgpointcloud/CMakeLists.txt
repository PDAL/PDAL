#
# PgPointCloud plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/pgpointcloud)
include_directories(${PROJECT_SOURCE_DIR}/io/las)

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
	io/PgReader.cpp
    )

    set(incs
	io/PgCommon.hpp
	io/PgReader.hpp
    )

    set(deps ${POSTGRESQL_LIBRARIES})
    PDAL_ADD_PLUGIN(pgpointcloud_reader_lib_name reader pgpointcloud "${srcs}" "${incs}" "${deps}")

    #
    # PgPointCloud Writer
    #
    set(srcs
	io/PgWriter.cpp
    )

    set(incs
	io/PgCommon.hpp
	io/PgWriter.hpp
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

        include_directories(${CMAKE_CURRENT_BINARY_DIR})

        set(deps ${pgpointcloud_reader_lib_name} ${pgpointcloud_writer_lib_name})
	PDAL_ADD_TEST(pgpointcloudtest FILES "${srcs}" LINK_WITH "${deps}")
    endif()
else()
    message(STATUS "Building without PgPointCloud support")
endif()
