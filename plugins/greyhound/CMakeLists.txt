#
# Greyhound plugin CMake configuration
#

include_directories(${JSONCPP_INCLUDE_DIR})
add_definitions(-DHAVE_JSONCPP=1)

set(srcs
    io/CompressionStream.cpp
    io/GreyhoundReader.cpp
    io/bbox.cpp
)

set(incs
    io/CompressionStream.hpp
    io/GreyhoundReader.hpp
    io/bbox.hpp
    io/point.hpp
)

PDAL_ADD_PLUGIN(libname reader greyhound
    FILES "${srcs}" "${incs}"
    LINK_WITH ${JSONCPP_LIBRARY})

set(srcs
    test/GreyhoundReaderTest.cpp
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

PDAL_ADD_TEST(greyhoundreadertest
    FILES "${srcs}"
    LINK_WITH ${libname} )
