cmake_minimum_required(VERSION 2.8.12)
project(WritingTutorial)

find_package(PDAL 1.0.0 REQUIRED CONFIG)
include_directories(${PDAL_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++11")
add_executable(tutorial tutorial.cpp)
target_link_libraries(tutorial ${PDAL_LIBRARIES})
