#
# Attribute filter CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/filters/attribute)

find_package(GEOS 3.3)
find_package(GDAL 1.9.0)
if (GEOS_FOUND AND GDAL_FOUND)
    message(STATUS "Building with Attribute support")

    include_directories(${GEOS_INCLUDE_DIR})
    include_directories(${GDAL_INCLUDE_DIR})

    set(srcs filters/AttributeFilter.cpp)
    set(incs filters/AttributeFilter.hpp)

    PDAL_ADD_PLUGIN(libname filter attribute
        FILES "${srcs}" "${incs}"
        LINK_WITH ${GEOS_LIBRARY} ${GDAL_LIBRARY})
else()
    message(STATUS "Building without Attribute support")
endif()
