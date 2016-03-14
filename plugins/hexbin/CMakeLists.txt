#
# HexBin plugin CMake configuration
#

include_directories(${ROOT_DIR}/vendor/pdalboost)

find_package(Hexer)
if (HEXER_FOUND)
    include_directories(${HEXER_INCLUDE_DIR})
    add_definitions(-DHAVE_HEXER=1)

    set(srcs filters/HexBin.cpp)
    set(incs filters/HexBin.hpp)

    PDAL_ADD_PLUGIN(libname filter hexbin
        FILES "${srcs}" "${incs}"
        LINK_WITH ${HEXER_LIBRARY})

    if (WITH_TESTS)
        PDAL_ADD_TEST(hexbintest
            FILES test/HexbinFilterTest.cpp
            LINK_WITH ${libname})
    endif()


    set(srcs kernel/DensityKernel.cpp kernel/OGR.cpp)
    set(incs kernel/DensityKernel.hpp kernel/OGR.hpp)

    PDAL_ADD_PLUGIN(density_kernel_libname kernel density
        FILES "${srcs}" "${incs}"
        LINK_WITH ${HEXER_LIBRARY} ${libname})

endif()
