#
# GeoWave support
#

# GeoWave support - optional, default=OFF
option(WITH_GeoWave "Choose if GeoWave support should be built" FALSE)
if(WITH_GeoWave)
    find_package(GeoWave 0.8.3)
    if(GEOWAVE_FOUND)    
		mark_as_advanced(CLEAR GeoWave_RUNTIME_JAR)
		add_definitions(-DGEOWAVE_RUNTIME_JAR=${GeoWave_RUNTIME_JAR})
        set(PDAL_HAVE_GeoWave 1)
    else()
		set(GeoWave_RUNTIME_JAR "")
        set(WITH_GeoWave FALSE)
    endif()
endif()
