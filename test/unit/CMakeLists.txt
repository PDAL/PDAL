###############################################################################
#
# test/unit/CMakeLists.txt controls building of PDAL unit tests suite
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
###############################################################################

configure_file(TestConfig.hpp.in "${CMAKE_CURRENT_BINARY_DIR}/TestConfig.hpp")

file(GLOB_RECURSE inFiles RELATIVE "${CMAKE_SOURCE_DIR}" "${CMAKE_SOURCE_DIR}/test/data/*.in")
foreach(infileName ${inFiles})
    string(REGEX REPLACE ".in\$" "" outfileName "${infileName}")
    configure_file("${CMAKE_SOURCE_DIR}/${infileName}"
        "${CMAKE_BINARY_DIR}/${outfileName}")
endforeach()

add_library(${PDAL_TEST_SUPPORT_OBJS} OBJECT Support.cpp)
add_dependencies(${PDAL_TEST_SUPPORT_OBJS} generate_dimension_hpp)
target_link_libraries(${PDAL_TEST_SUPPORT_OBJS}
    PRIVATE
        ${PDAL_LIB_NAME}
        GTest::gtest
)
target_include_directories(${PDAL_TEST_SUPPORT_OBJS}
    PRIVATE
        ${ROOT_DIR}
        ${PROJECT_BINARY_DIR}/include
        ${PROJECT_BINARY_DIR}/test/unit
        ${PDAL_INCLUDE_DIR}
)
if(NOT USE_EXTERNAL_GTEST)
    target_compile_options(${PDAL_TEST_SUPPORT_OBJS} BEFORE
        PRIVATE
            "-iquote${ROOT_DIR}/vendor/gtest/include"
    )
endif()
PDAL_TARGET_COMPILE_SETTINGS(${PDAL_TEST_SUPPORT_OBJS})

PDAL_ADD_TEST(pdal_io_copc_remote_reader_test
    FILES
        io/CopcRemoteReaderTest.cpp
        ${PDAL_SRC_DIR}/private/OGRSpec.cpp
    INCLUDES
        ${NLOHMANN_INCLUDE_DIR}
)

set_tests_properties(pdal_io_copc_remote_reader_test PROPERTIES TIMEOUT 300)

