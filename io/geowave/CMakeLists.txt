#
# GeoWave driver CMake configuration
#

set(objs "")

#
# GeoWave Reader
#
set(srcs
    GeoWaveReader.cpp
)

set(incs
    GeoWaveReader.hpp
)

PDAL_ADD_DRIVER(reader GeoWave "${srcs}" "${incs}" reader_objs)
set(objs ${objs} ${reader_objs})

#
# GeoWave Writer
#
set(srcs
    GeoWaveWriter.cpp
)

set(incs
    GeoWaveWriter.hpp
)

PDAL_ADD_DRIVER(writer GeoWave "${srcs}" "${incs}" writer_objs)
set(objs ${objs} ${writer_objs})

set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objs} PARENT_SCOPE)
