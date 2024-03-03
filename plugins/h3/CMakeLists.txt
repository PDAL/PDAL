cmake_minimum_required(VERSION 3.5)

if (STANDALONE)
    project(pdal-h3)
    find_package(PDAL REQUIRED)
    include("${CMAKE_SOURCE_DIR}/../../cmake/standalone.cmake" )
endif(STANDALONE)


include(${PDAL_CMAKE_DIR}/h3.cmake)
if (NOT H3_FOUND)
    message(FATAL_ERROR "H3 can't be found. Can't build H3 support")
endif()



PDAL_ADD_PLUGIN(h3_filter_libname filter h3
    FILES
        filters/H3Filter.cpp
    LINK_WITH
        ${PDAL_LIBRARIES}
        h3::h3
    INCLUDES
)

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_filters_h3_test
        FILES
            test/H3FilterTest.cpp
    LINK_WITH
        ${PDAL_LIBRARIES}
        h3::h3
        ${h3_filter_libname}
    )



endif(WITH_TESTS)
