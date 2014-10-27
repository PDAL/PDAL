#
# Points2grid plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugin/p2g)

find_package(Points2Grid REQUIRED)
if (P2G_FOUND)
    message(STATUS "Building with Points2grid support")

    include_directories(${P2G_INCLUDE_DIR})
    add_definitions(-DHAVE_P2G=1)

    #
    # Points2grid Writer
    #
    set(srcs
	drivers/P2gWriter.cpp
    )

    set(incs
	drivers/P2gWriter.hpp
    )

    set(P2G_WRITER_LIB_NAME pdal_plugin_writer_p2g)
    PDAL_ADD_PLUGIN(${P2G_WRITER_LIB_NAME} writer p2g ${srcs} ${incs})
    target_link_libraries(${P2G_WRITER_LIB_NAME} ${PDAL_LINKAGE} ${P2G_LIBRARY})
else()
    message(STATUS "Building without Points2grid support")
endif()
