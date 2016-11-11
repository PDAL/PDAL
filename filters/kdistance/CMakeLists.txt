PDAL_ADD_DRIVER(filter kdistance "KDistanceFilter.cpp" "KDistanceFilter.hpp" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
