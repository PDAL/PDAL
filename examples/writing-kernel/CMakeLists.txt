cmake_minimum_required(VERSION 2.8.12)
project(KernelTutorial)

find_package(PDAL 1.0.0 REQUIRED CONFIG)

set(CMAKE_CXX_FLAGS "-std=c++11")
add_library(pdal_plugin_kernel_mykernel SHARED MyKernel.cpp)
target_link_libraries(pdal_plugin_kernel_mykernel PRIVATE ${PDAL_LIBRARIES})
target_include_directories(pdal_plugin_kernel_mykernel PRIVATE
    ${PDAL_INCLUDE_DIRS}
    /Users/chambbj/loki/pdal/repo/kernels)
