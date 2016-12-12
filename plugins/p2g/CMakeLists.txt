#
# Points2grid plugin CMake configuration
#
find_package(P2G)
if (P2G_FOUND)
    PDAL_ADD_PLUGIN(libname writer p2g
        FILES
            io/P2gWriter.cpp
        LINK_WITH
            ${P2G_LIBRARY}
    )
    target_compile_definitions(${libname} PRIVATE -DHAVE_P2G=1)
    target_include_directories(${libname} PRIVATE ${P2G_INCLUDE_DIR})
endif()
