cmake_minimum_required(VERSION 3.6)
project(BatchStreamer)

find_package(PDAL 2.0.0 REQUIRED CONFIG)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(batch_streamer batch_streamer.cpp)

target_link_libraries(batch_streamer PRIVATE ${PDAL_LIBRARIES} PRIVATE Threads::Threads)
target_include_directories(batch_streamer PRIVATE
    ${PDAL_INCLUDE_DIRS}
    ${PDAL_INCLUDE_DIRS}/pdal)

