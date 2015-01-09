#
# Merge filter CMake configuration
#

#
# Merge Filter
#
set(srcs
)

set(incs
    MergeFilter.hpp
)

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
    DESTINATION "${PDAL_INCLUDE_INSTALL_DIR}"
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
