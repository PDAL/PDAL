#
# Terrasolid driver CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/drivers/terrasolid)

#
# Terrasolid Reader
#
set(srcs
    TerrasolidReader.cpp
)

set(incs
    TerrasolidReader.hpp
)

PDAL_ADD_DRIVER(reader terrasolid "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
