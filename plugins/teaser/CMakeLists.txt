#cmake_minimum_required(VERSION 3.1...3.15)
#project(pdalteaser)

find_package(OpenMP QUIET REQUIRED)
set_package_properties(OpenMP PROPERTIES
    TYPE OPTIONAL
    URL "https://www.openmp.org"
    PURPOSE "TEASER++ support")
mark_as_advanced(CLEAR OpenMP_INCLUDE_DIRS)
mark_as_advanced(CLEAR OpenMP_LIBRARIES)
if(NOT OpenMP_CXX_FOUND)
    message(FATAL_ERROR "This project requires the OpenMP library.")
endif()

find_package(teaserpp REQUIRED)
set_package_properties(teaserpp PROPERTIES
    TYPE OPTIONAL
    URL "https://teaser.readthedocs.io/en/latest/"
    PURPOSE "TEASER++ support")
mark_as_advanced(CLEAR teaserpp_INCLUDE_DIRS)
mark_as_advanced(CLEAR teaserpp_LIBRARIES)
if (NOT teaserpp_FOUND)
    message(FATAL_ERROR "This project requires the TEASER++ library.")
endif()

#find_package(PDAL REQUIRED CONFIG)
#if (NOT PDAL_FOUND)
#  message(FATAL_ERROR "This project requires the PDAL library, and will not be compiled.")
#endif()

find_package(PCL 1.9 REQUIRED COMPONENTS common features kdtree)
set_package_properties(PCL PROPERTIES
    TYPE OPTIONAL
    URL "https://pointclouds.org/"
    PURPOSE "TEASER++ support")
mark_as_advanced(CLEAR PCL_INCLUDE_DIRS)
mark_as_advanced(CLEAR PCL_LIBRARIES)
if (NOT PCL_FOUND)
    message(FATAL_ERROR "This project requires the Point Cloud Library.")
endif()

#include_directories(${PDAL_INCLUDE_DIRS})
#add_library(pdal_plugin_filter_teaser SHARED TeaserFilter.cpp)
#link_directories(${PDAL_LIBRARY_DIRS})
#target_link_libraries(pdal_plugin_filter_teaser ${PDAL_LIBRARIES} teaserpp::teaser_registration teaserpp::teaser_features OpenMP::OpenMP_CXX ${PCL_LIBRARIES})

PDAL_ADD_PLUGIN(filter_libname filter teaser
    FILES
        filters/TeaserFilter.cpp
    LINK_WITH
        teaserpp::teaser_registration
	teaserpp::teaser_features
	OpenMP::OpenMP_CXX
	${PCL_LIBRARIES}
    INCLUDES
        ${CMAKE_CURRENT_LIST_DIR}
)

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_filters_teaser_test
        FILES
            test/TeaserFilterTest.cpp
        LINK_WITH
            ${filter_libname}
        INCLUDES
            "${PDAL_VENDOR_DIR}/eigen"
    )
endif()
