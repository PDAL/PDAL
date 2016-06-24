PDAL_ADD_DRIVER(filter smrf "SMRFilter.cpp" "SMRFilter.hpp" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
