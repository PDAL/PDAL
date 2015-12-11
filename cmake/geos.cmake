#
# GEOS (optional)
#
find_package(GEOS QUIET 3.3)
set_package_properties(GEOS PROPERTIES TYPE OPTIONAL
    PURPOSE "Provides general purpose geometry support")
if (GEOS_FOUND)
    include_directories("${GEOS_INCLUDE_DIR}")
    set(PDAL_HAVE_GEOS 1)
else()
    set(GEOS_LIBRARY "")
endif()
