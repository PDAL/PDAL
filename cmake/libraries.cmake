# Build shared libraries by default.

set(PDAL_LIB_TYPE "SHARED")
if (WIN32)
    set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_IMPORT_LIBRARY_SUFFIX})
endif()
