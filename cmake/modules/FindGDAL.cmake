# - Find GDAL (gdal.h, GDAL library)
# This module defines
#  GDAL_INCLUDE_DIRS, directory(s) containing headers
#  GDAL_LIBRARIES, path to libzstd shared library
#  GDAL_FOUND, whether GDAL has been found

find_path(GDAL_INCLUDE_DIR NAMES gdal.h
    PATH_PREFIXES gdal)
mark_as_advanced(GDAL_INCLUDE_DIR)

find_library(GDAL_LIBRARY NAMES gdal)
mark_as_advanced(GDAL_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GDAL
    REQUIRED_VARS GDAL_LIBRARY GDAL_INCLUDE_DIR)

if (GDAL_FOUND)
    set(GDAL_LIBRARIES ${GDAL_LIBRARY})
    set(GDAL_INCLUDE_DIRS ${GDAL_INCLUDE_DIR})
endif()
