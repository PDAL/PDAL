PDAL_ADD_DRIVER(filter computerange "ComputeRangeFilter.cpp" "ComputeRangeFilter.hpp" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
