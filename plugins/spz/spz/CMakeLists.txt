#
# PDAL SPZ library config
#
# This can be deleted when we bump to a newer SPZ version
# (which have their own CMakeLists.txt)

project(libspz)

find_package(ZLIB REQUIRED)

file(GLOB SPZ_SRCS
    src/cc/*.cc
)

file(GLOB SPZ_HEADERS
    src/cc/*.h
)

add_library(libspz STATIC ${SPZ_SRCS})

target_link_libraries(libspz PRIVATE ZLIB::ZLIB)

set_target_properties(libspz PROPERTIES
    PUBLIC_HEADER "${SPZ_HEADERS}"
    POSITION_INDEPENDENT_CODE ON
)
