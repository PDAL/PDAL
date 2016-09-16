PDAL_ADD_DRIVER(filter lof "LOFFilter.cpp" "LOFFilter.hpp" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
