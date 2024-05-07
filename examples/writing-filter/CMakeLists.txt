cmake_minimum_required(VERSION 3.13)
project(FilterTutorial)

find_package(PDAL 2.5 REQUIRED CONFIG)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_library(pdal_plugin_filter_myfilter SHARED MyFilter.cpp)
target_link_libraries(pdal_plugin_filter_myfilter PRIVATE ${PDAL_LIBRARIES})
target_include_directories(pdal_plugin_filter_myfilter PRIVATE ${PDAL_INCLUDE_DIRS})
target_link_directories(pdal_plugin_filter_myfilter PRIVATE ${PDAL_LIBRARY_DIRS})
