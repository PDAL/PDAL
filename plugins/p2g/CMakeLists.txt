#
# Points2grid plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/p2g)

find_package(Points2Grid)
if (P2G_FOUND)
    include_directories(${P2G_INCLUDE_DIR})
    add_definitions(-DHAVE_P2G=1)

    set(srcs io/P2gWriter.cpp)
    set(incs io/P2gWriter.hpp)

    PDAL_ADD_PLUGIN(libname writer p2g
        FILES "${srcs}" "${incs}"
        LINK_WITH ${P2G_LIBRARY})
endif()
