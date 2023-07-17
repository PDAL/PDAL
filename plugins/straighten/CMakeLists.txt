cmake_minimum_required(VERSION 3.5)

if (STANDALONE)
    project(pdal-straighten)
    include(../../cmake/standalone.cmake NO_POLICY_SCOPE)
endif(STANDALONE)

PDAL_ADD_PLUGIN(libname filter straighten
    FILES
        filters/StraightenFilter.cpp
        filters/Polyline.cpp
        filters/Utils.cpp
    LINK_WITH
        ${PDAL_LIBRARIES}
        ${GDAL_LIBRARIES}
    INCLUDES
        ${PDAL_VENDOR_DIR}/eigen
)

