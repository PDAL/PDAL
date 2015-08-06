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

PDAL_ADD_DRIVER(kernel tindex "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
