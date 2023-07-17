cmake_minimum_required(VERSION 3.5)

if (STANDALONE)
    project(pdal-georeference)
    include(../../cmake/standalone.cmake NO_POLICY_SCOPE)
endif(STANDALONE)

find_package(GeographicLib 2.1.1 REQUIRED)

PDAL_ADD_PLUGIN(libname filter georeference
    FILES
        filters/GeoreferenceFilter.cpp
        filters/Trajectory.cpp
    LINK_WITH
        ${PDAL_LIBRARIES}
        ${GeographicLib_LIBRARIES} 
    INCLUDES
        ${PDAL_VENDOR_DIR}/eigen
        ${GeographicLib_INCLUDE_DIRS}
)
