#
# Delta kernel CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/kernels)
include_directories(${PROJECT_SOURCE_DIR}/kernels/delta)

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
