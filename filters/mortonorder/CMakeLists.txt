#
# MortonOrder filter CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/filters/mortonorder)

#
# MortonOrder Filter
#
set(srcs
    MortonOrderFilter.cpp
)

set(incs
    MortonOrderFilter.hpp
)

PDAL_ADD_DRIVER(filter mortonorder "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
