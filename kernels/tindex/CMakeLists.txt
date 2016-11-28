#
# TIndex kernel CMake configuration
#

#
# TIndex Kernel
#

set(srcs
    TIndexKernel.cpp
)

set(incs
    TIndexKernel.hpp
)

PDAL_ADD_KERNEL(tindex "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
