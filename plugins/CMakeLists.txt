if(WITH_GREYHOUND)
    add_subdirectory(greyhound)
endif()

if(WITH_ICEBRIDGE)
    add_subdirectory(icebridge)
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
