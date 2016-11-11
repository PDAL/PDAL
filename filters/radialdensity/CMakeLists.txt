PDAL_ADD_DRIVER(filter radialdensity "RadialDensityFilter.cpp" "RadialDensityFilter.hpp" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
