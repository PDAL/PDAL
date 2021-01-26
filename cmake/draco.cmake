#
option(WITH_DRACO "Choose if Draco support should be built" TRUE)
find_package(Draco EXACT 1.3.6)

if (WITH_DRACO)
    set_package_properties(Draco PROPERTIES TYPE RECOMMENDED
        PURPOSE "Provides Draco compression")
    if(Draco_FOUND)
        include_directories(${draco_INCLUDE_DIR})
        set(CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES}
            "draco")
        set(PDAL_HAVE_DRACO 1)
        set(BUILD_PLUGIN_DRACO 1)
        set(DRACO_LIBRARY "draco")
    else()
        set(WITH_DRACO FALSE)
    endif()
else()
        set(WITH_DRACO FALSE)
endif()
