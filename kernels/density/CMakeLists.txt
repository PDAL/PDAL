#
# Delta kernel CMake configuration
#

#
# Delta Kernel
#
set(srcs
    DensityKernel.cpp
    OGR.cpp
)

set(incs
    DensityKernel.hpp
    OGR.hpp
)

PDAL_ADD_DRIVER(kernel density "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
