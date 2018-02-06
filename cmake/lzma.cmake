#
# LZMA support
#
option(WITH_LZMA
    "Build support for compression/decompression with LZMA" FALSE)
if (WITH_LZMA)
    find_package(LibLZMA REQUIRED)
    set_package_properties(LibLZMA PROPERTIES TYPE REQUIRED
            PURPOSE "General compression support")
    if(LIBLZMA_FOUND)
        set(CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES}
            "${LIBLZMA_LIBRARIES}")
        include_directories(${LIBLZMA_INCLUDE_DIRS})
        mark_as_advanced(CLEAR LIBLZMA_INCLUDE_DIRS)
        mark_as_advanced(CLEAR LIBLZMA_LIBRARIES)
        set(PDAL_HAVE_LZMA 1)
    endif()
endif()
