#
# Terrasolid driver CMake configuration
#

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
