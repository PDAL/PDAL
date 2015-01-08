
add_subdirectory(delta)
add_subdirectory(diff)
add_subdirectory(info)
add_subdirectory(pipeline)
add_subdirectory(random)
add_subdirectory(sort)
add_subdirectory(translate)

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

if (NOT WIN32)
    add_definitions("-fPIC")
endif()
add_library(kernelcommon OBJECT ${srcs} ${incs})
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} $<TARGET_OBJECTS:kernelcommon> PARENT_SCOPE)
