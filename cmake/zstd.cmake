#
# LZMA support
#
option(WITH_ZSTD
    "Build support for compression/decompression with Zstd.")
if (WITH_ZSTD)
    find_package(ZSTD REQUIRED)
    set_package_properties(ZSTD PROPERTIES TYPE REQUIRED
        PURPOSE "General compression support")
    if (ZSTD_FOUND)
        set(CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES}
            "${ZSTD_STATIC_LIB}")
        mark_as_advanced(CLEAR ZSTD_INCLUDE_DIRS)
        mark_as_advanced(CLEAR ZSTD_LIBRARIES)
        set(PDAL_HAVE_ZSTD 1)
    endif(ZSTD_FOUND)
endif(WITH_ZSTD)
