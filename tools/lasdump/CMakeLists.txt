#
# The seemingly strange arrangement of this cmake file allows you to build
# lasdump as a standalone.
#
cmake_minimum_required(VERSION 3.5)

add_executable(lasdump
    Dumper.cpp
    Header.cpp
)
target_link_libraries(lasdump PRIVATE
    ${PDAL_UTIL_LIB_NAME}
    ${PDAL_LAZPERF_LIB_NAME}
)
target_include_directories(lasdump PRIVATE
    ${PDAL_VENDOR_DIR}
    ${PDAL_INCLUDE_DIR})
PDAL_TARGET_COMPILE_SETTINGS(lasdump)

