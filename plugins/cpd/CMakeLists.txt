find_package(Cpd 0.3 REQUIRED)

set_package_properties(Cpd PROPERTIES
    DESCRIPTION "Coherent Point Drift"
    URL "https://github.com/gadomski/cpd"
    TYPE OPTIONAL
    PURPOSE "Register two point sets using the Coherent Point Drift algorithm"
    )

PDAL_ADD_PLUGIN(cpd_kernel_lib_name kernel cpd
    FILES kernel/Cpd.cpp
    LINK_WITH Cpd::Library-C++
    )
target_include_directories(${cpd_kernel_lib_name} PRIVATE
    ${ROOT_DIR}
    ${CMAKE_CURRENT_LIST_DIR} )

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_plugins_cpd_kernel_test
        FILES test/CpdKernelTest.cpp
        LINK_WITH Cpd::Library-C++
        )
    target_include_directories(pdal_plugins_cpd_kernel_test
        PRIVATE
        ${ROOT_DIR}
        ${CMAKE_CURRENT_LIST_DIR}
        )
endif()
