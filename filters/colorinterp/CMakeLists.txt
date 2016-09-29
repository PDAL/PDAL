#
# Colorinterp filter CMake configuration
#

#
# Colorinterp Filter
#
set(srcs
    ColorinterpFilter.cpp
)

set(incs
    ColorinterpFilter.hpp
)

PDAL_ADD_DRIVER(filter colorinterp "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
