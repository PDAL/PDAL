cmake_minimum_required(VERSION 3.13)

if (STANDALONE)
    project(pdal-trajectory)
    include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/standalone.cmake" NO_POLICY_SCOPE)
    find_package(PDAL REQUIRED)
endif(STANDALONE)


find_package(Ceres REQUIRED)

PDAL_ADD_PLUGIN(libname filter trajectory
    FILES
        filters/Trajectory.cpp
        filters/PulseCollection.cpp
        filters/SplineFit.cpp
    LINK_WITH
        ${CERES_LIBRARIES}
        ${PDAL_LIBRARIES}
    INCLUDES
        ${PDAL_VENDOR_EIGEN_DIR}
)

if (WITH_TRAJECTORY_TEST)
if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_filters_traj_test
        FILES
            test/TrajectoryTest.cpp
    )
endif()
endif()
