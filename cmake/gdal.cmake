#
# GDAL/OGR support (required)
#
find_package(GDAL 2.2.0)
set_package_properties(GDAL PROPERTIES TYPE REQUIRED
    PURPOSE "Provides general purpose raster, vector, and reference system support")
if (GDAL_FOUND)
    string(COMPARE GREATER ${GDAL_VERSION} "3.0.0" GDAL_3)
    mark_as_advanced(CLEAR GDAL_INCLUDE_DIR)
    mark_as_advanced(CLEAR GDAL_LIBRARY)
else()
    message(FATAL_ERROR "GDAL support is required")
endif()
