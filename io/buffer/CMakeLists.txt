#
# Buffer driver CMake configuration
#

#
# Buffer Reader
#

set(incs
    BufferReader.hpp
)

PDAL_ADD_DRIVER(reader buffer "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
