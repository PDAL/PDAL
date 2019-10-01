
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
    INCLUDES
        ${NLOHMANN_INCLUDE_DIR}
        ${RDB_INCLUDE_CPP_DIR}
)
target_include_directories(${libname} PUBLIC "${PDAL_VENDOR_DIR}/eigen")

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
            ${libname}
        INCLUDES
            ${PROJECT_BINARY_DIR}/plugins/rdb/test
            ${PROJECT_SOURCE_DIR}/plugins/rdb/io
            ${RDB_INCLUDE_CPP_DIR}
    )
endif()
endif()
