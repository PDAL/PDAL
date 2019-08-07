#
# GDAL/OGR support (required)
#
find_package(GDAL 2.2.0)
set_package_properties(GDAL PROPERTIES TYPE REQUIRED
    PURPOSE "Provides general purpose raster, vector, and reference system support")
if (GDAL_FOUND)
    # if we have GDAL_VERSION (as a version type, not a string)
    # don't run gdal-config
    if ( ${GDAL_VERSION})
        # cmake is dumb. if we test NOT ${GDAL_CONFIG} it is true when GDAL_CONFIG
        # is set to a value. I don't know why.
    else()
        execute_process(COMMAND ${GDAL_CONFIG} "--version" OUTPUT_VARIABLE GDAL_VERSION)
    endif()
    string(COMPARE GREATER ${GDAL_VERSION} "3.0.0" GDAL_3)
    mark_as_advanced(CLEAR GDAL_INCLUDE_DIR)
    mark_as_advanced(CLEAR GDAL_LIBRARY)
else()
    message(FATAL_ERROR "GDAL support is required")
endif()
