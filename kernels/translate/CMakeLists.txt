#
# Translate kernel CMake configuration
#

#
# Translate Kernel
#
set(srcs
    TranslateKernel.cpp
)

set(incs
    TranslateKernel.hpp
)

PDAL_ADD_DRIVER(kernel translate "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
