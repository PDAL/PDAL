#
# Pipeline kernel CMake configuration
#

#
# Pipeline Kernel
#
set(srcs
    PipelineKernel.cpp
)

set(incs
    PipelineKernel.hpp
)

PDAL_ADD_KERNEL(pipeline "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
