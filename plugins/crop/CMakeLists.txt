#
# Crop filter CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/filters/crop)

find_package(GEOS 3.3)
if (GEOS_FOUND)
    message(STATUS "Building with Crop support")

    include_directories(${GEOS_INCLUDE_DIR})

    #
    # Crop Filter
    #
    set(srcs
        filters/CropFilter.cpp
    )

    set(incs
        filters/CropFilter.hpp
    )

    set(deps ${GEOS_LIBRARY})
    PDAL_ADD_PLUGIN(crop_filter_lib_name filter crop "${srcs}" "${incs}" "${deps}")

    #
    # Crop tests
    #
    set(srcs
        test/CropFilterTest.cpp
    )

    PDAL_ADD_TEST(croptest "${srcs}" ${crop_filter_lib_name})
else()
    message(STATUS "Building without Crop support")
endif()
