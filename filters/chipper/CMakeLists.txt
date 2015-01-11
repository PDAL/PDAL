#
# Chipper filter CMake configuration
#

#
# Chipper Filter
#
set(srcs
    ChipperFilter.cpp
)

set(incs
    ChipperFilter.hpp
)

PDAL_ADD_DRIVER(filter chipper "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
