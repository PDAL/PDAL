set(srcs MergeFilter.cpp)
set(incs MergeFilter.hpp)

PDAL_ADD_DRIVER(filter merge "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
