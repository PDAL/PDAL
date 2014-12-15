#
# HexBin plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/hexbin)
include_directories(${PROJECT_SOURCE_DIR}/io/las)

find_package(Hexer)
if (HEXER_FOUND)
    message(STATUS "Building with Hexer support")

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
else()
    message(STATUS "Building without Hexer support")
endif()
