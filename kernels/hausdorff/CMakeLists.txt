PDAL_ADD_DRIVER(kernel hausdorff "HausdorffKernel.cpp" "HausdorffKernel.hpp" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
