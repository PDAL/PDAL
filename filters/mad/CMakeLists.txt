PDAL_ADD_DRIVER(filter mad "MADFilter.cpp" "MADFilter.hpp" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
