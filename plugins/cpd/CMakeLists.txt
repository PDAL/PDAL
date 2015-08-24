# CPD plugin CMake configuration
#
# CPD is Coherent Point Drift: https://github.com/gadomski/cpd

find_package(CPD 0.2 REQUIRED)

set_package_properties(CPD PROPERTIES
    DESCRIPTION "Coherent Point Drift"
    URL "https://github.com/gadomski/cpd"
    TYPE OPTIONAL
    PURPOSE "Run the coherent point drift registration/change algorithm on two datasets via a kernel"
    )

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}
    include
    ${CPD_INCLUDE_DIRS}
    )

set(src
    kernel/Cpd.cpp
    )

set(deps
    cpd
    )

PDAL_ADD_PLUGIN(cpd_kernel_lib_name kernel cpd
    FILES ${src}
    LINK_WITH ${deps}
    )

if (WITH_TESTS)
    include_directories(
        ${PROJECT_SOURCE_DIR}/test/unit
        )
    PDAL_ADD_TEST(pdal_plugins_cpd_kernel_test
        FILES test/CpdKernelTest.cpp
        LINK_WITH ${deps}
        )
endif(WITH_TESTS)
