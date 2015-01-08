#
# Decimation filter CMake configuration
#

#
# Decimation Filter
#
set(srcs
    DecimationFilter.cpp
)

set(incs
    DecimationFilter.hpp
)

PDAL_ADD_DRIVER(filter decimation "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
