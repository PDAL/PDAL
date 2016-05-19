PDAL_ADD_DRIVER(filter estimaterank "EstimateRankFilter.cpp" "EstimateRankFilter.hpp" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
