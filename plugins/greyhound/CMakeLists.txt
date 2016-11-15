#
# Greyhound plugin CMake configuration
#

include_directories(${PDAL_JSONCPP_INCLUDE_DIR})
add_definitions(-DHAVE_JSONCPP=1)

set(srcs
    io/CompressionStream.cpp
    io/GreyhoundReader.cpp
    io/bounds.cpp
    io/pool.cpp
)

set(incs
    io/CompressionStream.hpp
    io/GreyhoundReader.hpp
    io/bounds.hpp
    io/dir.hpp
    io/point.hpp
    io/pool.hpp
)

PDAL_ADD_PLUGIN(libname reader greyhound
    FILES "${srcs}" "${incs}"
    LINK_WITH ${PDAL_JSONCPP_LIB_NAME})

if (WITH_TESTS)
    set(srcs
        test/GreyhoundReaderTest.cpp
    )

    include_directories(${CMAKE_CURRENT_BINARY_DIR})

    PDAL_ADD_TEST(greyhoundreadertest
        FILES "${srcs}"
        LINK_WITH ${libname} )
endif()
