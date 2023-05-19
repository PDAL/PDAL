
if (STANDALONE)
cmake_minimum_required(VERSION 3.5)
set(ROOT_DIR "../../${PROJECT_SOURCE_DIR}")
include(../../cmake/common.cmake NO_POLICY_SCOPE)
include(FeatureSummary)


find_package(PDAL COMPONENTS REQUIRED)
endif(STANDALONE)

find_package(rdb COMPONENTS REQUIRED)
set_package_properties(rdb PROPERTIES
    TYPE REQUIRED
    PURPOSE "Read data from RIEGL databases"
    )

PDAL_ADD_PLUGIN(libname reader rdb
    FILES
        io/RdbPointcloud.cpp
        io/RdbReader.cpp
    LINK_WITH
        rdbcpp
        ${PDAL_LIBRARIES}
    INCLUDES
        ${NLOHMANN_INCLUDE_DIR}
        ${RDB_INCLUDE_CPP_DIR}
        ${RDB_INCLUDE_C_DIR}
        ${PDAL_INCLUDE_DIRS}
)
target_include_directories(${libname} PRIVATE "${PDAL_VENDOR_DIR}/eigen")
target_include_directories(${libname} PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../../vendor/nlohmann/")
target_include_directories(${libname} PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../../vendor/eigen/")

# windows requires explicit linking against
# rdbcpp
if (WIN32)
	set(RDB_LIBRARY "rdbcpp")
endif()

set(RDB_TEST_NAME pdal_io_rdb_reader_test)
option(BUILD_RDBLIB_TESTS "Build rdblib tests" ON)
if (WITH_TESTS)
if (BUILD_RDBLIB_TESTS)
    configure_file(
        test/Config.hpp.in
        "${CMAKE_CURRENT_BINARY_DIR}/test/Config.hpp"
    )

    PDAL_ADD_TEST(${RDB_TEST_NAME}
        FILES
            test/RdbReaderTest.cpp
        LINK_WITH
            ${libname} ${RDB_LIBRARY}
        INCLUDES
            ${PROJECT_BINARY_DIR}/plugins/rdb/test
            ${PROJECT_SOURCE_DIR}/plugins/rdb/io
            ${RDB_INCLUDE_CPP_DIR}
            ${RDB_INCLUDE_C_DIR}
            ${PDAL_VENDOR_DIR}/eigen

    )
endif()
endif()
