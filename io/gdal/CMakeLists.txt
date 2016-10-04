set(src
    GDALReader.cpp
    GDALWriter.cpp
    GDALGrid.cpp
)

PDAL_ADD_DRIVER(reader gdal "${src}" "${incs}" objs)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objs} PARENT_SCOPE)
