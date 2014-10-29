#
# Hexer plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/hexer)

find_package(Hexer REQUIRED)
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

    set(HEXER_LIB_NAME pdal_plugin_filter_hexer)
    PDAL_ADD_PLUGIN(${HEXER_LIB_NAME} filter hexer ${srcs} ${incs})
    target_link_libraries(${HEXER_LIB_NAME} ${PDAL_LINKAGE} ${HEXER_LIBRARY})

    #
    # HexBin tests
    #
    set(srcs
        test/HexbinFilterTest.cpp
    )

    PDAL_ADD_TEST(hexertest ${srcs})
else()
    message(STATUS "Building without Hexer support")
endif()
