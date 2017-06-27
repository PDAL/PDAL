set(Cpd_VERSION 0.5)

find_package(Cpd ${Cpd_VERSION} REQUIRED)
option(BUILD_PLUGIN_CPD "Build Coherent Point Drift support" ${Cpd_FOUND})

set(files filters/CpdFilter.cpp)
set(include_dirs "${CMAKE_CURRENT_LIST_DIR}" "${PDAL_VENDOR_DIR}/eigen")

PDAL_ADD_PLUGIN(filter_libname filter cpd
    FILES filters/CpdFilter.cpp
    LINK_WITH Cpd::Library-C++
    )
target_include_directories(${filter_libname} PRIVATE "${include_dirs}")

if(${WITH_TESTS})
    PDAL_ADD_TEST(pdal_filters_cpd_test
        FILES test/CpdFilterTest.cpp
        LINK_WITH ${filter_libname}
        )
    target_include_directories(pdal_filters_cpd_test PRIVATE "${include_dirs}")
endif()
