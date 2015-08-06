#
# Merge kernel CMake configuration
#

#
# Merge Kernel
#
set(srcs
    MergeKernel.cpp
)

set(incs
    MergeKernel.hpp
)

PDAL_ADD_DRIVER(kernel merge "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
