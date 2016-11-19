# Translate Kernel
PDAL_ADD_KERNEL(translate TranslateKernel.cpp TranslateKernel.hpp objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
