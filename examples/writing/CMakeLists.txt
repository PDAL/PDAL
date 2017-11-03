cmake_minimum_required(VERSION 3.6)
project(WritingTutorial)

find_package(PDAL 1.6.0 REQUIRED CONFIG)

add_executable(tutorial tutorial.cpp)

target_link_libraries(tutorial PRIVATE ${PDAL_LIBRARIES})
target_include_directories(tutorial PRIVATE
    ${PDAL_INCLUDE_DIRS}
    ${PDAL_INCLUDE_DIRS}/pdal)

