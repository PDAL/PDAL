#
# HexBin plugin CMake configuration
#

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
endif()
