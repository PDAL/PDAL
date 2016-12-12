#
# HexBin plugin CMake configuration
#

find_package(Hexer REQUIRED)
if (HEXER_FOUND)
    add_definitions(-DHAVE_HEXER=1)

    PDAL_ADD_PLUGIN(libname filter hexbin
        FILES
            filters/HexBin.cpp
        LINK_WITH
            ${HEXER_LIBRARY})
    target_include_directories(${libname} PRIVATE
        ${PDAL_VENDOR_DIR}/pdalboost
        ${HEXER_INCLUDE_DIR})

    if (WITH_TESTS)
        PDAL_ADD_TEST(hexbintest
            FILES
                test/HexbinFilterTest.cpp
            LINK_WITH
                ${libname})
    endif()


    PDAL_ADD_PLUGIN(density_kernel_libname kernel density
        FILES
            kernel/DensityKernel.cpp
            kernel/OGR.cpp
        LINK_WITH
            ${HEXER_LIBRARY}
            ${libname})

endif()
