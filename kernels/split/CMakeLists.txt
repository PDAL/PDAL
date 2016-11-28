#
# Split kernel CMake configuration
#

#
# Split Kernel
#
set(srcs
    SplitKernel.cpp
)

set(incs
    SplitKernel.hpp
)

PDAL_ADD_KERNEL(split "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
