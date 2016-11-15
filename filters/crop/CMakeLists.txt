#
# Crop filter CMake configuration
#

#
# Crop Filter
#
set(srcs
    CropFilter.cpp
    Point.cpp
)

set(incs
    CropFilter.hpp
    Point.hpp
)

PDAL_ADD_DRIVER(filter crop "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
