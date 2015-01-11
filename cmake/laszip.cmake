#
# LASZIP support
#
option(WITH_LASZIP "Choose if LASzip support should be built" TRUE)
option(WITH_STATIC_LASZIP "Choose if LASzip should be statically linked" FALSE)
mark_as_advanced(WITH_STATIC_LASZIP)
if(WITH_LASZIP)
    find_package(LASzip QUIET 1.0.1)
    set_package_properties(LASzip PROPERTIES TYPE RECOMMENDED
        PURPOSE "Provides LASzip compression")
    if(LASZIP_FOUND)
        set(CMAKE_REQUIRED_LIBRARIES "${LASZIP_LIBRARY}")
        include_directories(${LASZIP_INCLUDE_DIR})
        mark_as_advanced(CLEAR LASZIP_INCLUDE_DIR)
        mark_as_advanced(CLEAR LASZIP_LIBRARY)
        mark_as_advanced(CLEAR LASZIP_VERSION)
        set(PDAL_HAVE_LASZIP 1)
    else()
        set(LASZIP_LIBRARY "")
        set(WITH_LASZIP FALSE)
    endif()
endif()
