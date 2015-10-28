#
# Divider filter CMake configuration
#

set(srcs
    DividerFilter.cpp
)

set(incs
    DividerFilter.hpp
)

PDAL_ADD_DRIVER(filter divider "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
