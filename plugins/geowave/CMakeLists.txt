#
# GeoWave driver CMake configuration
#

include(${PDAL_CMAKE_DIR}/jni.cmake)
include(${PDAL_CMAKE_DIR}/jace.cmake)
include(${PDAL_CMAKE_DIR}/geowave.cmake)

set(GEOWAVE_LIBRARIES "")
set(GEOWAVE_LIBRARIES ${GEOWAVE_LIBRARIES} ${Boost_LIBRARIES})
set(GEOWAVE_LIBRARIES ${GEOWAVE_LIBRARIES} ${JNI_LIBRARIES})
set(GEOWAVE_LIBRARIES ${GEOWAVE_LIBRARIES} ${JACE_LIBRARY})
set(GEOWAVE_LIBRARIES ${GEOWAVE_LIBRARIES} ${JAVA_AWT_LIBRARY})
set(GEOWAVE_LIBRARIES ${GEOWAVE_LIBRARIES} ${JAVA_JVM_LIBRARY})

set(objs "")

#
# GeoWave Reader
#
set(srcs
    io/GeoWaveReader.cpp
)

set(incs
    io/GeoWaveReader.hpp
)

PDAL_ADD_PLUGIN(reader_libname reader geowave 
    FILES "${srcs}" "${incs}" 
    LINK_WITH ${GEOWAVE_LIBRARIES})

#
# GeoWave Writer
#
set(srcs
    io/GeoWaveWriter.cpp
)

set(incs
    io/GeoWaveWriter.hpp
)

PDAL_ADD_PLUGIN(writer_libname writer geowave 
    FILES "${srcs}" "${incs}" 
    LINK_WITH ${GEOWAVE_LIBRARIES})

