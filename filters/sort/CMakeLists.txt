#
# Sort filter CMake configuration
#

#
# Sort Filter
#
set(srcs
)

set(incs
    SortFilter.hpp
)

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
    DESTINATION "${PDAL_INCLUDE_INSTALL_DIR}"
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
