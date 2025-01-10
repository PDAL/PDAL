
cmake_minimum_required(VERSION 3.13)

add_executable(lasdump
    Dumper.cpp
    Header.cpp
    ${PDAL_UTIL_DIR}/Bounds.cpp
    ${PDAL_UTIL_DIR}/FileUtils.cpp
    ${PDAL_UTIL_DIR}/Utils.cpp
)

target_link_libraries(lasdump PRIVATE
    ${PDAL_LAZPERF_LIB_NAME}
)
target_include_directories(lasdump PRIVATE
    ${PDAL_VENDOR_DIR}
    ${PDAL_INCLUDE_DIR}
    ${NLOHMANN_INCLUDE_DIR}
    ${UTFCPP_INCLUDE_DIR}
)

#
# For byte-ordering functions
#
if (WIN32)
    target_link_libraries(lasdump PRIVATE wsock32 ws2_32)
endif()

PDAL_TARGET_COMPILE_SETTINGS(lasdump)

