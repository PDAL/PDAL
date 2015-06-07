#
# Grid kernel CMake configuration
#

#
# Grid Kernel
#
set(srcs
    GridKernel.cpp
)

set(incs
    GridKernel.hpp
)

PDAL_ADD_DRIVER(kernel grid "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
