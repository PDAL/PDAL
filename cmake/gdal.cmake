#
# GDAL/OGR support (required)
#
find_package(GDAL 1.9.0)
set_package_properties(GDAL PROPERTIES TYPE REQUIRED
    PURPOSE "Provides general purpose raster, vector, and reference system support")
if (GDAL_FOUND)
    mark_as_advanced(CLEAR GDAL_INCLUDE_DIRS)
    mark_as_advanced(CLEAR GDAL_LIBRARIES)
else()
    message(FATAL_ERROR "GDAL support is required")
endif()
