#
# Delta kernel CMake configuration
#

#
# Delta Kernel
#
set(srcs
    DeltaKernel.cpp
)

set(incs
    DeltaKernel.hpp
)

PDAL_ADD_DRIVER(kernel delta "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
