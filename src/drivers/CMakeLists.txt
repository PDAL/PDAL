###############################################################################
#
# src/drivers/CMakeLists.txt controls building of PDAL library
#
# Copyright (c) 2011 Howard Butler <hobu@hobu.net>
#
###############################################################################


add_subdirectory(faux)
add_subdirectory(las)
add_subdirectory(liblas)

if(CARIS_FOUND)
    add_subdirectory(caris)
endif()

if(MRSID_FOUND)
    add_subdirectory(mrsid)
endif()

if(ORACLE_FOUND AND GDAL_FOUND)
    add_subdirectory(oci)
endif()

if (P2G_FOUND)
    add_subdirectory(p2g)
endif()

