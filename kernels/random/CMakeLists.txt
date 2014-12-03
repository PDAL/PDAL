#
# Random kernel CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/kernels)
include_directories(${PROJECT_SOURCE_DIR}/kernels/random)

#
# Random Kernel
#
set(srcs
    RandomKernel.cpp
)

set(incs
    RandomKernel.hpp
)

PDAL_ADD_DRIVER(kernel random "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
