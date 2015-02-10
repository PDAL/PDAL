#
# JNI support
#

# JNI support - optional, default=OFF
if(BUILD_PLUGIN_GEOWAVE)
    find_package(JNI)
    if(JNI_FOUND) 
        set(CMAKE_REQUIRED_LIBRARIES "${JNI_LIBRARIES}")
        include_directories(${JNI_INCLUDE_DIRS})
        mark_as_advanced(CLEAR JNI_INCLUDE_DIRS)
        mark_as_advanced(CLEAR JNI_LIBRARIES)
        add_definitions(-DJAVA_EXECUTABLE_PATH=${Java_JAVA_EXECUTABLE})
        set(PDAL_HAVE_JNI 1)
    else()
        set(JNI_LIBRARIES "")
        set(BUILD_PLUGIN_GEOWAVE FALSE)
    endif()
endif()
