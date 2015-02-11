#
# GeoWave support
#

# GeoWave support - optional, default=OFF
option(BUILD_PLUGIN_GEOWAVE "Choose if GeoWave support should be built" FALSE)
if(BUILD_PLUGIN_GEOWAVE)
    find_package(GeoWave 0.8.3)
    if(GEOWAVE_FOUND)    
        mark_as_advanced(CLEAR GeoWave_RUNTIME_JAR)
        add_definitions(-DGEOWAVE_RUNTIME_JAR=${GEOWAVE_RUNTIME_JAR})
        set(PDAL_HAVE_GEOWAVE 1)
    else()
        set(GEOWAVE_RUNTIME_JAR "")
        set(BUILD_PLUGIN_GEOWAVE FALSE)
    endif()
endif()
