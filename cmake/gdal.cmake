#
# GDAL/OGR support (required)
#
function(gdal_find_version _version)
    file(READ ${GDAL_INCLUDE_DIR}/gdal_version.h versionfile)
    string(REGEX MATCH "GDAL_VERSION_MAJOR[\t ]+([0-9]+)" _ ${versionfile})
    set(MAJOR ${CMAKE_MATCH_1})
    string(REGEX MATCH "GDAL_VERSION_MINOR[\t ]+([0-9]+)" _ ${versionfile})
    set(MINOR ${CMAKE_MATCH_1})
    string(REGEX MATCH "GDAL_VERSION_REV[\t ]+([0-9]+)" _ ${versionfile})
    set(REV ${CMAKE_MATCH_1})
    set(${_version} ${MAJOR}.${MINOR}.${REV} PARENT_SCOPE)
endfunction(gdal_find_version)

find_package(GDAL 3.0 REQUIRED)
set_package_properties(GDAL PROPERTIES TYPE REQUIRED
    PURPOSE "Provides general purpose raster, vector, and reference system support")
if (GDAL_FOUND)
    gdal_find_version(GDAL_VERSION)
    #
    # Older versions of FindGDAL.cmake don't properly set GDAL_VERSION
    #
    if (GDAL_VERSION VERSION_LESS 3.0.0)
        message(FATAL_ERROR
            "Found GDAL version ${GDAL_VERSION}.  Version 3.0+ is required")
    endif()
    mark_as_advanced(CLEAR GDAL_INCLUDE_DIR)
    mark_as_advanced(CLEAR GDAL_LIBRARY)
else()
    message(FATAL_ERROR "GDAL support is required")
endif()
