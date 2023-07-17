cmake_minimum_required(VERSION 3.5)

if (STANDALONE)
    project(pdal-georeference)
    include(../../cmake/standalone.cmake NO_POLICY_SCOPE)
endif(STANDALONE)


PDAL_ADD_PLUGIN(libname filter georeference
    FILES
        filters/GeoreferenceFilter.cpp
        filters/Trajectory.cpp
        filters/LocalCartesian.cpp
    LINK_WITH
        ${PDAL_LIBRARIES}
    INCLUDES
        ${PDAL_VENDOR_DIR}/eigen
)
