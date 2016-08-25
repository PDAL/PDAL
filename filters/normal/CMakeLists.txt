PDAL_ADD_DRIVER(filter normal "NormalFilter.cpp" "NormalFilter.hpp" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
