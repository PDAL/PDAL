#
# Ferry filter CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/filters/ferry)

#
# Ferry Filter
#
set(srcs
    FerryFilter.cpp
)

set(incs
    FerryFilter.hpp
)

PDAL_ADD_DRIVER(filter ferry "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
