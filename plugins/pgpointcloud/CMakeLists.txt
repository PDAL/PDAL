cmake_minimum_required(VERSION 3.13)

if (STANDALONE)
    project(pdal-pgpointcloud)
    include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/standalone.cmake" )
    include(${PDAL_CMAKE_DIR}/arbiter.cmake)
    include(${PDAL_CMAKE_DIR}/nlohmann.cmake)
    include(${PDAL_CMAKE_DIR}/libxml2.cmake)
    add_subdirectory("${PROJECT_SOURCE_DIR}/../../vendor/arbiter" "vendor/arbiter")
    find_package(PDAL REQUIRED)
endif(STANDALONE)

#
# PgPointCloud plugin CMake configuration
#
include(${PDAL_CMAKE_DIR}/postgres.cmake)
include(${PDAL_CMAKE_DIR}/libxml2.cmake)

#
# PgPointCloud Reader
#
PDAL_ADD_PLUGIN(reader_libname reader pgpointcloud
    FILES
        io/PgReader.cpp
    LINK_WITH
        PostgreSQL::PostgreSQL
        ${LIBXML2_LIBRARIES}
        ${PDAL_LIBRARIES}
    INCLUDES
        ${LIBXML2_INCLUDE_DIR}
)

#
# PgPointCloud Writer
#
PDAL_ADD_PLUGIN(writer_libname writer pgpointcloud
    FILES
        io/PgWriter.cpp
    LINK_WITH
        PostgreSQL::PostgreSQL
        ${LIBXML2_LIBRARIES}
        ${PDAL_LIBRARIES}
    INCLUDES
        ${LIBXML2_INCLUDE_DIR}
)
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
        ${CMAKE_CURRENT_BINARY_DIR}/include/Pgtest-Support.hpp
    )

	PDAL_ADD_TEST(pgpointcloudtest
        FILES
            test/PgpointcloudTest.cpp
        LINK_WITH
            ${reader_libname}
            ${writer_libname}
            PostgreSQL::PostgreSQL
        INCLUDES
            ${CMAKE_CURRENT_BINARY_DIR}/include
    )
endif()
