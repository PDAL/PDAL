if(BUILD_PLUGIN_GREYHOUND)
    add_subdirectory(greyhound)
endif()

if(BUILD_PLUGIN_ICEBRIDGE)
    add_subdirectory(icebridge)
endif()

if(BUILD_PLUGIN_MRSID)
    add_subdirectory(mrsid)
endif()

if(BUILD_PLUGIN_NITF)
    add_subdirectory(nitf)
endif()

if(BUILD_PLUGIN_OCI)
    add_subdirectory(oci)
endif()

if(BUILD_PLUGIN_P2G)
    add_subdirectory(p2g)
endif()

if(BUILD_PLUGIN_PCL)
    add_subdirectory(pcl)
endif()

if(BUILD_PLUGIN_PGPOINTCLOUD)
    add_subdirectory(pgpointcloud)
endif()

if(BUILD_PLUGIN_SQLITE)
    add_subdirectory(sqlite)
endif()
