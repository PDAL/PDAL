cmake_minimum_required(VERSION 2.8.12)
project(FilterTutorial)

find_package(PDAL 1.9.0 REQUIRED CONFIG)

set (CMAKE_CXX_STANDARD 11) 
add_library(pdal_plugin_filter_myfilter SHARED MyFilter.cpp)
target_link_libraries(pdal_plugin_filter_myfilter PRIVATE ${PDAL_LIBRARIES})
target_include_directories(pdal_plugin_filter_myfilter PRIVATE ${PDAL_INCLUDE_DIRS})
