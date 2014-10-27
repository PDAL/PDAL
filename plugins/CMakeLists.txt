if(WITH_GREYHOUND)
    add_subdirectory(greyhound)
endif()

if(WITH_ICEBRIDGE)
    add_subdirectory(icebridge)
endif()

if(WITH_MRSID)
    add_subdirectory(mrsid)
endif()

if(WITH_NITF)
    add_subdirectory(nitf)
endif()

if(WITH_ORACLE)
    add_subdirectory(oci)
endif()

if(WITH_P2G)
    add_subdirectory(p2g)
endif()

if(WITH_PCL)
    add_subdirectory(pcl)
endif()

if(WITH_PGPOINTCLOUD)
    add_subdirectory(pgpointcloud)
endif()

if(WITH_SQLITE)
    add_subdirectory(sqlite)
endif()
