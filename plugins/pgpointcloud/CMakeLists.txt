#
# PgPointCloud plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/pgpointcloud)

find_package(PostgreSQL REQUIRED)
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

    set(PGPOINTCLOUD_READER_LIB_NAME pdal_plugin_reader_pgpointcloud)
    PDAL_ADD_PLUGIN(${PGPOINTCLOUD_READER_LIB_NAME} reader pgpointcloud ${srcs} ${incs})
    target_link_libraries(${PGPOINTCLOUD_READER_LIB_NAME} ${PDAL_LINKAGE} ${POSTGRESQL_LIBRARIES})

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

    set(PGPOINTCLOUD_WRITER_LIB_NAME pdal_plugin_writer_pgpointcloud)
    PDAL_ADD_PLUGIN(${PGPOINTCLOUD_WRITER_LIB_NAME} writer pgpointcloud ${srcs} ${incs})
    target_link_libraries(${PGPOINTCLOUD_WRITER_LIB_NAME} ${PDAL_LINKAGE} ${POSTGRESQL_LIBRARIES})

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

        PDAL_ADD_TEST(pgpointcloudtest ${srcs})
    endif()
else()
    message(STATUS "Building without PgPointCloud support")
endif()
