cmake_minimum_required(VERSION 2.8.12)
project(ReaderTutorial)

find_package(PDAL 1.6.0 REQUIRED CONFIG)

if (NOT WIN32)
    set(CMAKE_CXX_FLAGS "-std=c++11")
endif()

add_library(pdal_plugin_reader_myreader SHARED MyReader.cpp)
target_link_libraries(pdal_plugin_reader_myreader PRIVATE ${PDAL_LIBRARIES})
target_include_directories(pdal_plugin_reader_myreader PRIVATE
                            ${PDAL_INCLUDE_DIRS})
