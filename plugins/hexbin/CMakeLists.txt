#
# HexBin plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/hexbin)

find_package(Hexer)
if (HEXER_FOUND)
    message(STATUS "Building with Hexer support")

    include_directories(${HEXER_INCLUDE_DIR})
    add_definitions(-DHAVE_HEXER=1)

    #
    # HexBin filter
    #
    set(srcs
        filters/HexBin.cpp
    )

    set(incs
        filters/HexBin.hpp
    )

    set(deps ${HEXER_LIBRARY})
    PDAL_ADD_PLUGIN(hexbin_filter_lib_name filter hexbin "${srcs}" "${incs}" "${deps}")

    #
    # HexBin tests
    #
    set(srcs
        test/HexbinFilterTest.cpp
    )

    PDAL_ADD_TEST(hexbintest "${srcs}")
else()
    message(STATUS "Building without Hexer support")
endif()
