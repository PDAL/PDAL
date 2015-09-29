#
# ILVIS2 driver CMake configuration
#

set(objs "")

#
# ILVIS2 Reader
#
set(srcs
    Ilvis2Reader.cpp
)

set(incs
    Ilvis2Reader.hpp
)

PDAL_ADD_DRIVER(reader ilvis2 "${srcs}" "${incs}" reader_objs)
set(objs ${objs} ${reader_objs})

set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objs} PARENT_SCOPE)
