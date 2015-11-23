set(src
    TIndexReader.cpp
)

set(inc
    TIndexReader.hpp
)

PDAL_ADD_DRIVER(reader tindex "${src}" "${inc}" objs)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objs} PARENT_SCOPE)
