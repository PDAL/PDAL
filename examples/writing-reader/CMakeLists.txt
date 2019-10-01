cmake_minimum_required(VERSION 2.8.12)
project(ReaderTutorial)

find_package(PDAL 2.0 REQUIRED CONFIG)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(pdal_plugin_reader_myreader SHARED MyReader.cpp)
target_link_libraries(pdal_plugin_reader_myreader PRIVATE ${PDAL_LIBRARIES})
target_include_directories(pdal_plugin_reader_myreader PRIVATE
                            ${PDAL_INCLUDE_DIRS})
target_link_directories(pdal_plugin_reader_myreader PRIVATE ${PDAL_LIBRARY_DIRS})
