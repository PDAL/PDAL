cmake_minimum_required(VERSION 2.8.12)
project(KernelTutorial)

find_package(PDAL 1.0.0 REQUIRED CONFIG)
include_directories(${PDAL_INCLUDE_DIRS})

include_directories(/Users/chambbj/loki/pdal/repo/kernels)

set(CMAKE_CXX_FLAGS "-std=c++11")
add_library(pdal_plugin_kernel_mykernel SHARED MyKernel.cpp)
target_link_libraries(pdal_plugin_kernel_mykernel ${PDAL_LIBRARIES})
