#
# GEOS (optional)
#
find_package(GEOS QUIET 3.3)
set_package_properties(GEOS PROPERTIES TYPE OPTIONAL
    PURPOSE "Provides general purpose geometry support")
if (GEOS_FOUND)
    find_file(GEOS_VERSION_H version.h "${GEOS_INCLUDE_DIR}/geos")
    if ("${GEOS_VERSION_H}" STREQUAL "GEOS_VERSION_H-NOTFOUND")
        set(GEOS_LIBRARY "")
        set(GEOS_FOUND FALSE)
    else()
        include_directories("${GEOS_INCLUDE_DIR}")
        set(PDAL_HAVE_GEOS 1)
    endif()
else()
    set(GEOS_LIBRARY "")
endif()
