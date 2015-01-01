#
# Attribute filter CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/filters/attribute)

find_package(GEOS QUIET 3.3)
set_package_properties(GEOS PROPERTIES PURPOSE "Enables attribute filter")

find_package(GDAL QUIET 1.9.0)
set_package_properties(GDAL PROPERTIES PURPOSE "Enables attribute filter")

if (GEOS_FOUND AND GDAL_FOUND)
    include_directories(${GEOS_INCLUDE_DIR})
    include_directories(${GDAL_INCLUDE_DIR})

    set(srcs filters/AttributeFilter.cpp)
    set(incs filters/AttributeFilter.hpp)

    PDAL_ADD_PLUGIN(libname filter attribute
        FILES "${srcs}" "${incs}"
        LINK_WITH ${GEOS_LIBRARY} ${GDAL_LIBRARY})
else()
    message(STATUS "BUILD_PLUGIN_ATTRIBUTE disabled because GEOS and/or GDAL was not found")
endif()
