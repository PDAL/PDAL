###############################################################################
#
# src/drivers/CMakeLists.txt controls building of libPC library
#
# Copyright (c) 2011 Howard Butler <hobu@hobu.net>
#
###############################################################################



add_subdirectory(liblas)
add_subdirectory(las)

if(ORACLE_FOUND AND GDAL_FOUND)
    add_subdirectory(oci)
endif()

