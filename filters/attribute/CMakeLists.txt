#
# Attribute filter CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/filters/attribute)

#
# Attribute Filter
#
set(srcs
    AttributeFilter.cpp
)

set(incs
    AttributeFilter.hpp
)

PDAL_ADD_DRIVER(filter attribute "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
