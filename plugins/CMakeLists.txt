include(${PDAL_CMAKE_DIR}/test.cmake)

add_subdirectory(faux)
if (BUILD_PLUGIN_TRAJECTORY)
    add_subdirectory(trajectory)
endif()


if(BUILD_PLUGIN_CPD)
    add_subdirectory(cpd)
endif(BUILD_PLUGIN_CPD)

if(BUILD_PLUGIN_DRACO)
    add_subdirectory(draco)
endif(BUILD_PLUGIN_DRACO)

if(BUILD_PLUGIN_ICEBRIDGE)
    if (NOT PDAL_HAVE_LIBXML2)
        message(FATAL_ERROR "Can't build icebridge plugin without libxml2")
    endif()
    add_subdirectory(icebridge)
endif()

if(BUILD_PLUGIN_HDF)
    add_subdirectory(hdf)
endif()

if(BUILD_PLUGIN_MATLAB)
    add_subdirectory(matlab)
endif()

if(BUILD_PLUGIN_MBIO)
    add_subdirectory(mbio)
endif()

if(BUILD_PLUGIN_NITF)
    add_subdirectory(nitf)
endif()

if(BUILD_PLUGIN_PGPOINTCLOUD)
    if (NOT PDAL_HAVE_LIBXML2)
        message(FATAL_ERROR "Can't build pgpointcloud plugin without libxml2")
    endif()
    add_subdirectory(pgpointcloud)
endif()

if(BUILD_PLUGIN_OPENSCENEGRAPH)
    add_subdirectory(openscenegraph)
endif()

if(BUILD_PLUGIN_RDBLIB)
    add_subdirectory(rdb)
endif(BUILD_PLUGIN_RDBLIB)

if(BUILD_PLUGIN_RIVLIB)
    add_subdirectory(rxp)
endif(BUILD_PLUGIN_RIVLIB)

if(BUILD_PLUGIN_TEASER)
    add_subdirectory(teaser)
endif(BUILD_PLUGIN_TEASER)

if(BUILD_PLUGIN_TILEDB)
    add_subdirectory(tiledb)
endif()

if(BUILD_PLUGIN_E57)
    add_subdirectory(e57)
endif()
