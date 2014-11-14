#
# Diff kernel CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/kernels)
include_directories(${PROJECT_SOURCE_DIR}/kernels/diff)

#
# Diff Kernel
#
set(srcs
    DiffKernel.cpp
)

set(incs
    DiffKernel.hpp
)

PDAL_ADD_DRIVER(kernel diff "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
