PDAL_ADD_DRIVER(filter outlier "OutlierFilter.cpp" "OutlierFilter.hpp" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
