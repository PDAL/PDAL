cmake_minimum_required(VERSION 3.6)
project(StreamerTutorial)

find_package(PDAL 2.0.0 REQUIRED CONFIG)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(streamer_tutorial streamer_tutorial.cpp)

target_link_libraries(streamer_tutorial PRIVATE ${PDAL_LIBRARIES})
target_include_directories(streamer_tutorial PRIVATE
    ${PDAL_INCLUDE_DIRS}
    ${PDAL_INCLUDE_DIRS}/pdal)

