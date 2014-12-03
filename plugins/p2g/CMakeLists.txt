#
# Points2grid plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/p2g)

find_package(Points2Grid)
if (P2G_FOUND)
    message(STATUS "Building with Points2grid support")

    include_directories(${P2G_INCLUDE_DIR})
    add_definitions(-DHAVE_P2G=1)

    #
    # Points2grid Writer
    #
    set(srcs
	io/P2gWriter.cpp
    )

    set(incs
	io/P2gWriter.hpp
    )

    set(deps ${P2G_LIBRARY})
    PDAL_ADD_PLUGIN(p2g_writer_lib_name writer p2g "${srcs}" "${incs}" "${deps}")
else()
    message(STATUS "Building without Points2grid support")
endif()
