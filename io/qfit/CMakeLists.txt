#
# QFIT driver CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/io/qfit)

#
# QFIT Reader
#
set(srcs
    QfitReader.cpp
)

set(incs
    QfitReader.hpp
)

PDAL_ADD_DRIVER(reader qfit "${srcs}" "${incs}" objs)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objs} PARENT_SCOPE)
