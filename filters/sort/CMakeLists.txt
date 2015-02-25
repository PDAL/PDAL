set(srcs SortFilter.cpp)
set(incs SortFilter.hpp)

PDAL_ADD_DRIVER(filter sort "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
