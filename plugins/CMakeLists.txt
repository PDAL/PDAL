if(BUILD_PLUGIN_ATTRIBUTE)
    add_subdirectory(attribute)
endif()

if(BUILD_PLUGIN_CPD)
    add_subdirectory(cpd)
endif(BUILD_PLUGIN_CPD)

if(BUILD_PLUGIN_GEOWAVE)
    add_subdirectory(geowave)
endif(BUILD_PLUGIN_GEOWAVE)

if(BUILD_PLUGIN_GREYHOUND)
    add_subdirectory(greyhound)
endif()

if(BUILD_PLUGIN_HEXBIN)
    add_subdirectory(hexbin)
endif()

if(BUILD_PLUGIN_ICEBRIDGE)
    add_subdirectory(icebridge)
endif()

if(BUILD_PLUGIN_MATLAB)
    add_subdirectory(matlab)
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

if(BUILD_PLUGIN_PYTHON)
    add_subdirectory(python)
endif()

if(BUILD_PLUGIN_RIVLIB)
    add_subdirectory(rxp)
endif(BUILD_PLUGIN_RIVLIB)

if(BUILD_PLUGIN_SQLITE)
    add_subdirectory(sqlite)
endif()

