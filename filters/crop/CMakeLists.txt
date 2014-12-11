#
# Crop filter CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/filters/crop)

#
# Crop Filter
#
set(srcs
    CropFilter.cpp
)

set(incs
    CropFilter.hpp
)

PDAL_ADD_DRIVER(filter crop "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
