#
# Reprojection filter CMake configuration
#

#
# Reprojection Filter
#
set(srcs
    ReprojectionFilter.cpp
)

set(incs
    ReprojectionFilter.hpp
)

PDAL_ADD_DRIVER(filter reprojection "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
