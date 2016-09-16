set(src
    PlyReader.cpp
    PlyWriter.cpp
    ${PROJECT_SOURCE_DIR}/vendor/rply/rply.c
)

set(inc
    PlyReader.hpp
    PlyWriter.hpp
    ${PROJECT_SOURCE_DIR}/vendor/rply/rply.h
)

PDAL_ADD_DRIVER(reader ply "${src}" "${inc}" objs)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objs} PARENT_SCOPE)
