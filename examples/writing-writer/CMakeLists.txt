cmake_minimum_required(VERSION 2.8.12)
project(WriterTutorial)

find_package(PDAL 1.0.0 REQUIRED CONFIG)
include_directories(${PDAL_INCLUDE_DIRS})

include_directories(/Users/chambbj/loki/pdal/repo/writers)

set(CMAKE_CXX_FLAGS "-std=c++11")
add_library(pdal_plugin_writer_mywriter SHARED MyWriter.cpp)
target_link_libraries(pdal_plugin_writer_mywriter ${PDAL_LIBRARIES})
