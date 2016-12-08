cmake_minimum_required(VERSION 2.8.12)
project(WritingTutorial)

find_package(PDAL 1.0.0 REQUIRED CONFIG)

set(CMAKE_CXX_FLAGS "-std=c++11")
add_executable(tutorial tutorial.cpp)
target_link_libraries(tutorial PRIVATE ${PDAL_LIBRARIES})
target_include_directories(tutorial PRIVATE
    ${PDAL_INCLUDE_DIRS}
    ${PDAL_INCLUDE_DIRS}/pdal)
