PDAL_ADD_DRIVER(filter approximatecoplanar "ApproximateCoplanarFilter.cpp" "ApproximateCoplanarFilter.hpp" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
