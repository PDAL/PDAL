set(srcs RangeFilter.cpp)
set(incs RangeFilter.hpp)

PDAL_ADD_DRIVER(filter range "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
