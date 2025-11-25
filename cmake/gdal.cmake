#
# GDAL/OGR support (required)
#

find_package(GDAL CONFIG REQUIRED)
set_package_properties(GDAL PROPERTIES TYPE REQUIRED
    PURPOSE "Provides general purpose raster, vector, and reference system support")
if(GDAL_VERSION VERSION_LESS "3.8")
  message(FATAL_ERROR "Required at least GDAL version 3.8, but found ${GDAL_VERSION}.")
endif()
