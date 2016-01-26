#
# GEOS (required)
#
find_package(GEOS QUIET 3.3)
set_package_properties(GEOS PROPERTIES TYPE REQUIRED
    PURPOSE "Provides general purpose geometry support")

include_directories("${GEOS_INCLUDE_DIR}")
