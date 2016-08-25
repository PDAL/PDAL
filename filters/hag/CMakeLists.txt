PDAL_ADD_DRIVER(filter hag "HAGFilter.cpp" "HAGFilter.hpp" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
