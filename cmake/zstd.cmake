#
# Zstd support
#
option(WITH_ZSTD
    "Build support for compression/decompression with Zstd." TRUE)
if (WITH_ZSTD)
    find_package(Zstd CONFIG QUIET)
    set_package_properties(ZSTD PROPERTIES TYPE
        PURPOSE "General compression support")
    if (ZSTD_FOUND)
        set(PDAL_HAVE_ZSTD 1)
    endif(ZSTD_FOUND)
endif(WITH_ZSTD)
