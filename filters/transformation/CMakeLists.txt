#
# Transformation filter CMake configuration
#

#
# Transformation Filter
#
set(srcs
    TransformationFilter.cpp
)

set(incs
    TransformationFilter.hpp
)

PDAL_ADD_DRIVER(filter transformation "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)

