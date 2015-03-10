set(srcs
    OptechReader.cpp
    )

set(incs
    OptechReader.hpp
    )

PDAL_ADD_DRIVER(reader optech "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
