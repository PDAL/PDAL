# Build shared libraries by default.

set(PDAL_LIB_TYPE "SHARED")
if (WIN32)
    set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_IMPORT_LIBRARY_SUFFIX})
endif()


# Must be changed if there is an ABI change.  This builds the SONAME
# that's embedded in the library itself.
#
set(PDAL_SOLIB_MAJOR 17)
#
# Change if there's a new release with no ABI change
#
set(PDAL_SOLIB_MINOR 1)
#
# Build number.  Increment for builds within minor versions.  Probably
# always 0 for PDAL
#
set(PDAL_SOLIB_BUILD 0)


set(PDAL_API_VERSION ${PDAL_SOLIB_MAJOR})
set(PDAL_BUILD_VERSION
    "${PDAL_SOLIB_MAJOR}.${PDAL_SOLIB_MINOR}.${PDAL_SOLIB_BUILD}")
