set(RXP_TEST_NAME rxptest)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugin/rxp)

find_package(RiVLib COMPONENTS scanlib REQUIRED)

include_directories(${RiVLib_INCLUDE_DIRS})
message(STATUS "Found RiVLib at: ${RiVLib_INCLUDE_DIRS}")

set(incs
    drivers/RxpReader.hpp
    )

set(srcs
    drivers/RxpReader.cpp
    )

set(deps
    ${RiVLib_SCANLIB_LIBRARY}
    )

PDAL_ADD_PLUGIN(rxp_reader_lib_name reader rxp "${srcs}" "${incs}" "${deps}")


if (BUILD_RIVLIB_TESTS)
    configure_file(
        test/Config.hpp.in
        "${CMAKE_CURRENT_BINARY_DIR}/test/Config.hpp"
        )

    include_directories(
        "${CMAKE_CURRENT_BINARY_DIR}/test"
        )

    set(test_srcs
        test/RxpReaderTest.cpp
        )

    set(test_deps
        ${libname}
        )

    PDAL_ADD_TEST(${RXP_TEST_NAME} ${test_srcs} ${rxp_reader_lib_name})

endif (BUILD_RIVLIB_TESTS)
