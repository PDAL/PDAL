#
# Sort filter CMake configuration
#

#
# Sort Filter
#
set(srcs
    SortFilter.cpp
)

set(incs
    SortFilter.hpp
)

#install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
#    DESTINATION "${PDAL_INCLUDE_INSTALL_DIR}"
#    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
#)

PDAL_ADD_DRIVER(filter sort "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
