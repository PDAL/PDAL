#
# Zstd support
#
option(WITH_ZSTD
    "Build support for compression/decompression with Zstd." TRUE)
if (WITH_ZSTD)
    find_package(ZSTD QUIET)
    set_package_properties(ZSTD PROPERTIES TYPE
        PURPOSE "General compression support")
    if (ZSTD_FOUND)
        set(CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES}
            "${ZSTD_STATIC_LIB}")
        mark_as_advanced(CLEAR ZSTD_INCLUDE_DIRS)
        mark_as_advanced(CLEAR ZSTD_LIBRARIES)
        set(PDAL_HAVE_ZSTD 1)
    else()
        set(ZSTD_LIBRARIES "")
        set(ZSTD_INCLUDE_DIRS "")
        set(WITH_ZSTD FALSE)
    endif(ZSTD_FOUND)
else()
        set(ZSTD_LIBRARIES "")
        set(WITH_ZSTD FALSE)
endif(WITH_ZSTD)
