set(RXP_TEST_NAME rxptest)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugin/rxp)

find_package(RiVLib COMPONENTS scanlib REQUIRED)

include_directories(${RiVLib_INCLUDE_DIRS})
message(STATUS "Found RiVLib at: ${RiVLib_INCLUDE_DIRS}")

set(incs io/RxpReader.hpp)
set(srcs io/RxpReader.cpp)

PDAL_ADD_PLUGIN(libname reader rxp
    FILES "${srcs}" "${incs}"
    LINK_WITH ${RiVLib_SCANLIB_LIBRARY})

if (BUILD_RIVLIB_TESTS)
    configure_file(
        test/Config.hpp.in
        "${CMAKE_CURRENT_BINARY_DIR}/test/Config.hpp"
    )

    include_directories("${CMAKE_CURRENT_BINARY_DIR}/test")
    include_directories(${PROJECT_SOURCE_DIR}/plugin/rxp/io)

    PDAL_ADD_TEST(${RXP_TEST_NAME}
        FILES test/RxpReaderTest.cpp
        LINK_WITH ${libname})
endif (BUILD_RIVLIB_TESTS)
