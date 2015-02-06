#
# JNI support
#

# JNI support - optional, default=OFF
option(WITH_JNI "Choose if JNI support should be built" FALSE)
if(WITH_JNI)
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
        set(WITH_JNI FALSE)
    endif()
endif()
