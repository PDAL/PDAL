
find_package(Ceres REQUIRED)

PDAL_ADD_PLUGIN(libname filter trajectory
    FILES
        filters/Trajectory.cpp
        filters/PulseCollection.cpp
        filters/SplineFit.cpp
    LINK_WITH
        ${CERES_LIBRARIES}
    INCLUDES
        ${PDAL_VENDOR_DIR}/eigen
)

