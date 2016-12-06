cmake_minimum_required(VERSION 2.8.12)
project(WriterTutorial)

find_package(PDAL 1.0.0 REQUIRED CONFIG)

set(CMAKE_CXX_FLAGS "-std=c++11")
add_library(pdal_plugin_writer_mywriter SHARED MyWriter.cpp)
target_link_libraries(pdal_plugin_writer_mywriter PRIVATE ${PDAL_LIBRARIES})
target_include_directories(pdal_plugin_writer_mywriter PRIVATE
    ${PDAL_INCLUDE_DIRS})
