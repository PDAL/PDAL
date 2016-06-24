PDAL_ADD_DRIVER(filter mongus "MongusFilter.cpp" "MongusFilter.hpp" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
