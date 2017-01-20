find_package(Cpd 0.5 REQUIRED)

set_package_properties(Cpd PROPERTIES
    DESCRIPTION "Coherent Point Drift"
    URL "https://github.com/gadomski/cpd"
    TYPE OPTIONAL
    PURPOSE "Register two point sets using the Coherent Point Drift algorithm"
    )

pdal_add_plugin(cpd_kernel_lib_name kernel cpd
    FILES kernel/CpdKernel.cpp
    LINK_WITH Cpd::Library-C++
    )
target_include_directories(${cpd_kernel_lib_name} PRIVATE ${CMAKE_CURRENT_LIST_DIR})
