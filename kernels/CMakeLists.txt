add_subdirectory(delta)
add_subdirectory(diff)
add_subdirectory(info)
add_subdirectory(pipeline)
add_subdirectory(random)
add_subdirectory(sort)
add_subdirectory(translate)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/io/buffer)
include_directories(${PROJECT_SOURCE_DIR}/kernels)
include_directories(${PROJECT_SOURCE_DIR}/kernels/delta)
include_directories(${PROJECT_SOURCE_DIR}/kernels/diff)
include_directories(${PROJECT_SOURCE_DIR}/kernels/info)
include_directories(${PROJECT_SOURCE_DIR}/kernels/pipeline)
include_directories(${PROJECT_SOURCE_DIR}/kernels/random)
include_directories(${PROJECT_SOURCE_DIR}/kernels/sort)
include_directories(${PROJECT_SOURCE_DIR}/kernels/translate)

set(srcs
  KernelFactory.cpp
  KernelInfo.cpp
  Kernel.cpp
  KernelSupport.cpp
)

set(incs
  KernelFactory.hpp
  KernelInfo.hpp
  Kernel.hpp
  KernelSupport.hpp
)

add_library(kernelcommon OBJECT ${srcs} ${incs})
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} $<TARGET_OBJECTS:kernelcommon> PARENT_SCOPE)
