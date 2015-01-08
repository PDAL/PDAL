#
# Colorization filter CMake configuration
#

#
# Colorization Filter
#
set(srcs
    ColorizationFilter.cpp
)

set(incs
    ColorizationFilter.hpp
)

PDAL_ADD_DRIVER(filter colorization "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
