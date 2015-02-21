set(RXP_TEST_NAME rxptest)

find_package(RiVLib COMPONENTS scanlib REQUIRED)
set_package_properties(RiVLib PROPERTIES
    TYPE REQUIRED
    PURPOSE "Read data from Riegl sensors"
    )

include_directories(${RiVLib_INCLUDE_DIRS})
message(STATUS "Found RiVLib at: ${RiVLib_INCLUDE_DIRS}")

set(incs
    io/RxpPointcloud.hpp
    io/RxpReader.hpp
    )

set(srcs
    io/RxpPointcloud.cpp
    io/RxpReader.cpp
    )

PDAL_ADD_PLUGIN(libname reader rxp
    FILES "${srcs}" "${incs}"
    LINK_WITH ${RiVLib_SCANLIB_LIBRARY})

if (BUILD_RIVLIB_TESTS)
    configure_file(
        test/Config.hpp.in
        "${CMAKE_CURRENT_BINARY_DIR}/test/Config.hpp"
    )

    include_directories("${CMAKE_CURRENT_BINARY_DIR}/test")
    include_directories(${PROJECT_SOURCE_DIR}/plugins/rxp/io)

    PDAL_ADD_TEST(${RXP_TEST_NAME}
        FILES test/RxpReaderTest.cpp
        LINK_WITH ${libname})
endif (BUILD_RIVLIB_TESTS)
