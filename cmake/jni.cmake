#
# JNI support
#

# JNI support - optional, default=OFF
if(BUILD_PLUGIN_GEOWAVE)
    find_package(JNI)
    if(JNI_FOUND) 
        set(CMAKE_REQUIRED_LIBRARIES "${JAVA_JVM_LIBRARY}")
        include_directories(${JAVA_INCLUDE_PATH})
        include_directories(${JAVA_INCLUDE_PATH2})
        mark_as_advanced(CLEAR JAVA_INCLUDE_PATH)
        mark_as_advanced(CLEAR JAVA_INCLUDE_PATH2)
        mark_as_advanced(CLEAR JAVA_JVM_LIBRARY)
        set(PDAL_HAVE_JNI 1)
    else()
        set(JAVA_INCLUDE_PATH "")
        set(JAVA_INCLUDE_PATH2 "")
        set(JAVA_JVM_LIBRARY "")
        set(BUILD_PLUGIN_GEOWAVE FALSE)
    endif()
endif()
