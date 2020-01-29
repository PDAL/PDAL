include(${PDAL_CMAKE_DIR}/test.cmake)

add_subdirectory(faux)
if(BUILD_PLUGIN_I3S)
    if(NOT ZLIB_FOUND)
        message(FATAL_ERROR "Can't build i3s and slpk plugins without zlib")
    endif()
    add_subdirectory(i3s)
endif()

if(BUILD_PLUGIN_CPD)
    add_subdirectory(cpd)
endif(BUILD_PLUGIN_CPD)

if(BUILD_PLUGIN_GEOWAVE)
    add_subdirectory(geowave)
endif(BUILD_PLUGIN_GEOWAVE)

if(BUILD_PLUGIN_ICEBRIDGE)
    if (NOT PDAL_HAVE_LIBXML2)
        message(FATAL_ERROR "Can't build icebridge plugin without libxml2")
    endif()
    add_subdirectory(icebridge)
endif()

if(BUILD_PLUGIN_MATLAB)
    add_subdirectory(matlab)
endif()

if(BUILD_PLUGIN_MBIO)
    add_subdirectory(mbio)
endif()

if(BUILD_PLUGIN_MRSID)
    add_subdirectory(mrsid)
endif()

if(BUILD_PLUGIN_NITF)
    add_subdirectory(nitf)
endif()

if(BUILD_PLUGIN_OCI)
    if (NOT PDAL_HAVE_LIBXML2)
        message(FATAL_ERROR "Can't build OCI plugin without libxml2")
    endif()
    add_subdirectory(oci)
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

if(BUILD_PLUGIN_SQLITE)
    if (NOT PDAL_HAVE_LIBXML2)
        message(FATAL_ERROR "Can't build sqlite plugin without libxml2")
    endif()
    add_subdirectory(sqlite)
endif()

if (BUILD_PLUGIN_FBX)
    add_subdirectory(fbx)
endif()

if(BUILD_PLUGIN_TILEDB)
    add_subdirectory(tiledb)
endif()

if(BUILD_PLUGIN_E57)
    add_subdirectory(e57)
endif()
