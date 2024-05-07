cmake_minimum_required(VERSION 3.6)
project(StreamerTutorial)

find_package(PDAL 2.0.0 REQUIRED CONFIG)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(reading_streamer reading_streamer.cpp)

target_link_libraries(reading_streamer PRIVATE ${PDAL_LIBRARIES})
target_include_directories(reading_streamer PRIVATE
    ${PDAL_INCLUDE_DIRS}
    ${PDAL_INCLUDE_DIRS}/pdal)

