#
# Info kernel CMake configuration
#

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
