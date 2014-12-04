#
# Info kernel CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/kernels)
include_directories(${PROJECT_SOURCE_DIR}/kernels/info)

#
# Info Kernel
#
set(srcs
    InfoKernel.cpp
)

set(incs
    InfoKernel.hpp
)

PDAL_ADD_DRIVER(kernel info "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
