# Omni Kernel
PDAL_ADD_DRIVER(kernel omni OmniKernel.cpp OmniKernel.hpp objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
