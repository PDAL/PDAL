#
# QFIT driver CMake configuration
#

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
