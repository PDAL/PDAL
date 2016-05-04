PDAL_ADD_DRIVER(filter pmf "PMFFilter.cpp" "PMFFilter.hpp" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
