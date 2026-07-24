#
# GDAL/OGR support (required)
#

find_package(GDAL CONFIG REQUIRED)
set_package_properties(GDAL PROPERTIES TYPE REQUIRED
    PURPOSE "Provides general purpose raster, vector, and reference system support")
if(GDAL_VERSION VERSION_LESS "3.8")
  message(FATAL_ERROR "Required at least GDAL version 3.8, but found ${GDAL_VERSION}.")
endif()

# Need to manually find the plugin path to see if OGR_Parquet is available.
get_target_property(GDAL_LIBRARY_PATH GDAL::GDAL LOCATION)
get_filename_component(GDAL_LIBRARY_DIR ${GDAL_LIBRARY_PATH} DIRECTORY)
set(GDAL_PLUGINS_PATHS
  "${GDAL_LIBRARY_DIR}/gdalplugins"
  "$ENV{GDAL_DRIVER_PATH}")
message(STATUS "GDAL_LIBRARY: ${GDAL_PLUGINS_PATHS}")

find_file(OGR_PARQUET_LIBRARY
  NAMES ogr_Parquet.so ogr_Parquet.dylib ogr_Parquet.dll
  PATHS ${GDAL_PLUGINS_PATHS})
if(OGR_PARQUET_LIBRARY)
  set(PDAL_HAVE_OGR_PARQUET 1)
endif()
