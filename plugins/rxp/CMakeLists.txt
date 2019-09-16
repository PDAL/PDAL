set(RXP_TEST_NAME rxptest)

find_package(RiVLib COMPONENTS scanlib REQUIRED)
set_package_properties(RiVLib PROPERTIES
    TYPE REQUIRED
    PURPOSE "Read data from RIEGL sensors"
    )

PDAL_ADD_PLUGIN(libname reader rxp
    FILES
        io/RxpPointcloud.cpp
        io/RxpReader.cpp
    LINK_WITH
        ${RiVLib_SCANLIB_LIBRARY}
    INCLUDES
        ${RiVLib_INCLUDE_DIRS}
)

option(BUILD_RIVLIB_TESTS "Build rivlib tests" ON)
if (WITH_TESTS)
if (BUILD_RIVLIB_TESTS)
    configure_file(
        test/Config.hpp.in
        "${CMAKE_CURRENT_BINARY_DIR}/test/Config.hpp"
    )

    PDAL_ADD_TEST(${RXP_TEST_NAME}
        FILES
            test/RxpReaderTest.cpp
        LINK_WITH
            ${libname}
        INCLUDES
            ${RiVLib_INCLUDE_DIRS}
            ${PROJECT_BINARY_DIR}/plugins/rxp/test
            ${PROJECT_SOURCE_DIR}/plugins/rxp/io
    )
endif()
endif()
