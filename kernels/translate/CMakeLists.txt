#
# Translate kernel CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/filters/reprojection)
include_directories(${PROJECT_SOURCE_DIR}/io/buffer)
include_directories(${PROJECT_SOURCE_DIR}/kernels)
include_directories(${PROJECT_SOURCE_DIR}/kernels/translate)

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
