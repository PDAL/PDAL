cmake_minimum_required(VERSION 2.8.12)
project(ReaderTutorial)

find_package(PDAL 1.0.0 REQUIRED CONFIG)
include_directories(${PDAL_INCLUDE_DIRS})

include_directories(/Users/chambbj/loki/pdal/repo/readers)

set(CMAKE_CXX_FLAGS "-std=c++11")
add_library(pdal_plugin_reader_myreader SHARED MyReader.cpp)
target_link_libraries(pdal_plugin_reader_myreader ${PDAL_LIBRARIES})
