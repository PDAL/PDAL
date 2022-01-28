
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

#
# For byte-ordering functions
#
if (WIN32)
    target_link_libraries(lasdump PRIVATE wsock32 ws2_32)
endif()

PDAL_TARGET_COMPILE_SETTINGS(lasdump)

