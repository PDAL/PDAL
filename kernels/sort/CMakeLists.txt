#
# Sort kernel CMake configuration
#

#
# Sort Kernel
#
set(srcs
    SortKernel.cpp
)

set(incs
    SortKernel.hpp
)

PDAL_ADD_KERNEL(sort "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
