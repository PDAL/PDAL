set(src
    GDALReader.cpp
)

set(inc
    GDALReader.hpp
)

PDAL_ADD_DRIVER(reader gdal "${src}" "${inc}" objs)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objs} PARENT_SCOPE)
