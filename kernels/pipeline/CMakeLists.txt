#
# Pipeline kernel CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/kernels)
include_directories(${PROJECT_SOURCE_DIR}/kernels/pipeline)

#
# Pipeline Kernel
#
set(srcs
    PipelineKernel.cpp
)

set(incs
    PipelineKernel.hpp
)

PDAL_ADD_DRIVER(kernel pipeline "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
