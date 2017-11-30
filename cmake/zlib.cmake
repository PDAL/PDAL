#
# ZLIB support
#
option(WITH_ZLIB
    "Build support for compression/decompression with zlib/deflate." TRUE)
if (WITH_ZLIB)
    find_package(ZLIB REQUIRED)
    set_package_properties(ZLIB PROPERTIES TYPE REQUIRED
        PURPOSE "Compression support in BPF")
    if(ZLIB_FOUND)
        set(CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES}
            "${ZLIB_LIBRARY}")
        include_directories(${ZLIB_INCLUDE_DIR})
        mark_as_advanced(CLEAR ZLIB_INCLUDE_DIR)
        mark_as_advanced(CLEAR ZLIB_LIBRARY)
        set(PDAL_HAVE_ZLIB 1)
    endif(ZLIB_FOUND)
endif(WITH_ZLIB)
