PDAL_ADD_DRIVER(filter sample "SampleFilter.cpp" "SampleFilter.hpp" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
