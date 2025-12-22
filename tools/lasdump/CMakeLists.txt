
cmake_minimum_required(VERSION 3.13)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(lasdump
    Dumper.cpp
    Header.cpp
    ${PDAL_UTIL_DIR}/VSIIO.cpp
    ${PDAL_UTIL_DIR}/Bounds.cpp
    ${PDAL_UTIL_DIR}/FileUtils.cpp
    ${PDAL_UTIL_DIR}/Utils.cpp
)

# Explicitly link and tell CMake to link libstdc++
target_link_libraries(lasdump PRIVATE
    ${PDAL_LAZPERF_LIB_NAME}
    GDAL::GDAL
    "$<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>>:-lstdc++fs>"
    "$<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>>:-lstdc++>"
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

