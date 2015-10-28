set(srcs RandomizeFilter.cpp)
set(incs RandomizeFilter.hpp)

PDAL_ADD_DRIVER(filter randomize "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
