#
# Gridder filter CMake configuration
#

#
# Gridder Filter
#
set(srcs
    GridderFilter.cpp
)

set(incs
    GridderFilter.hpp
)

PDAL_ADD_DRIVER(filter gridder "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
