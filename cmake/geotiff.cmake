#
# GeoTIFF support
#
option(WITH_GEOTIFF "Choose if GeoTIFF support should be built" TRUE)
if (WITH_GEOTIFF)
    find_package(GeoTIFF QUIET 1.3.0)
    set_package_properties(GeoTIFF PROPERTIES TYPE OPTIONAL)
    if (GEOTIFF_FOUND)
        include_directories("${GEOTIFF_INCLUDE_DIR}")
        set(PDAL_HAVE_LIBGEOTIFF 1)
        mark_as_advanced(CLEAR TIFF_INCLUDE_DIR)
        mark_as_advanced(CLEAR TIFF_LIBRARY)
        if (WIN32)
            set(TIFF_NAMES libtiff_i)
        endif(WIN32)
    else()
        set(WITH_GEOTIFF FALSE)
    endif()
endif()
