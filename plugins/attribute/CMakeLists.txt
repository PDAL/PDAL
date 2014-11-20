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

    #
    # Attribute Filter
    #
    set(srcs
	filters/AttributeFilter.cpp
    )

    set(incs
	filters/AttributeFilter.hpp
    )

    set(deps ${GEOS_LIBRARY} ${GDAL_LIBRARY})
    PDAL_ADD_PLUGIN(attribute_filter_lib_name filter attribute "${srcs}" "${incs}" "${deps}")
else()
    message(STATUS "Building without Attribute support")
endif()
