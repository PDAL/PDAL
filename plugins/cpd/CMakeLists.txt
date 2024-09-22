cmake_minimum_required(VERSION 3.13)

if (STANDALONE)
    project(pdal-cpd)
    include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/standalone.cmake" )
    find_package(PDAL REQUIRED)
endif(STANDALONE)

set(Cpd_VERSION 0.5)

find_package(Cpd ${Cpd_VERSION} REQUIRED CONFIG)
find_package(Fgt REQUIRED CONFIG)
option(BUILD_PLUGIN_CPD "Build Coherent Point Drift support" ${Cpd_FOUND})

set(files filters/CpdFilter.cpp)
set(include_dirs "${CMAKE_CURRENT_LIST_DIR}" "${EIGEN_INCLUDE_DIRS}")

PDAL_ADD_PLUGIN(filter_libname filter cpd
    FILES
        filters/CpdFilter.cpp
    LINK_WITH
        Cpd::Library-C++
        ${PDAL_LIBRARIES}
    INCLUDES
        ${EIGEN_INCLUDE_DIRS}
        ${CMAKE_CURRENT_LIST_DIR}
    )

if(${WITH_TESTS})
    PDAL_ADD_TEST(pdal_filters_cpd_test
        FILES
            test/CpdFilterTest.cpp
        LINK_WITH
            ${filter_libname}
            Cpd::Library-C++
        INCLUDES
            "${include_dirs}"
        )
endif()
