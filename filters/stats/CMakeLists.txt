#
# Stats filter CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/filters/stats)

#
# Stats Filter
#
set(srcs
    StatsFilter.cpp
)

set(incs
    StatsFilter.hpp
)

PDAL_ADD_DRIVER(filter stats "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
