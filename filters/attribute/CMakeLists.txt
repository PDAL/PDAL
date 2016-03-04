#
# Attribute filter CMake configuration
#

set(srcs AttributeFilter.cpp)
set(incs AttributeFilter.hpp)

PDAL_ADD_DRIVER(filter attribute "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
