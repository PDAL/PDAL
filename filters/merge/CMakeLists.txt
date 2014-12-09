#
# Merge filter CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/filters/merge)

#
# Merge Filter
#
set(srcs
)

set(incs
    MergeFilter.hpp
)

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
    DESTINATION "${PDAL_INCLUDE_DIR}"
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
