# Build shared libraries by default.

option(PDAL_BUILD_STATIC "Build PDAL as a static library" OFF)
if (PDAL_BUILD_STATIC)
  set(PDAL_LIB_TYPE "STATIC")
  set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_STATIC_LIBRARY_SUFFIX})
else ()
  set(PDAL_LIB_TYPE "SHARED")
  if (WIN32)
    set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_IMPORT_LIBRARY_SUFFIX})
  endif()
endif()
mark_as_advanced(PDAL_BUILD_STATIC)
