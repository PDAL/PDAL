#
# PCL plugin CMake configuration
#
# The PCL integration still has a few warts that need to be resolved:
#
#     1. src/kernel/Kernel.cpp requires PCL for invoking the PCLVisualizer with
#        two point buffers.
#
#        This is currently disabled (commented out), and should be somehow
#        fully integrated as a plugin behavior.
#
#     2. PCL unit tests still live within main test directory.
#
#        We should be able to move plugin tests to plugin directories without
#        too much hassle. PCL unit tests will pass with a warning if the PCL
#        plugins are not installed in PDAL_DRIVER_PATH (just assumes you didn't
#        intend to test with PCL support).
#

#------------------------------------------------------------------------------
# settings for PCL
#------------------------------------------------------------------------------

find_package(PCL QUIET 1.7 REQUIRED)

#
# PCL has compiler issues.
#
if (PDAL_COMPILER_CLANG)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-c99-extensions")
endif()

set_package_properties(PCL PROPERTIES DESCRIPTION "Point Cloud Library"
    URL "http://pointclouds.org" TYPE RECOMMENDED
    PURPOSE "Enables PCD reader/writer, PCLVisualizer, PCLBlock filter, ICP filter, and ground, pcl, smooth, and view kernels")

set(INCLUDE_DIRS
    ${ROOT_DIR}
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_LIST_DIR}
    )

add_definitions(${PCL_DEFINITIONS})
if (NOT WIN32)
    add_definitions("-fvisibility-inlines-hidden")
endif()

# PCL's configuration clobbers Boost find_package - do it again
find_package(Boost QUIET 1.52 COMPONENTS program_options iostreams filesystem system thread)

#
# PCD Reader
#
PDAL_ADD_PLUGIN(pcd_reader_libname reader pcd
    FILES
        io/PcdCommon.cpp
        io/PcdReader.cpp
    LINK_WITH ${PCL_LIBRARIES})
target_include_directories(${pcd_reader_libname} PRIVATE
    ${INCLUDE_DIRS})

#
# PCD Writer
#
PDAL_ADD_PLUGIN(pcd_writer_libname writer pcd
    FILES
        io/PcdCommon.cpp
        io/PcdWriter.cpp
    LINK_WITH ${PCL_LIBRARIES})
target_include_directories(${pcd_writer_libname} PRIVATE
    ${INCLUDE_DIRS})

#
# PCLBlock Filter
#
PDAL_ADD_PLUGIN(pclblock_libname filter pclblock
    FILES
        pipeline/PCLPipeline.cpp
        filters/PCLBlock.cpp
    LINK_WITH
        ${PCL_LIBRARIES}
        ${PDAL_JSONCPP_LIB_NAME}
)
target_include_directories(${pclblock_libname}
    PRIVATE
      ${INCLUDE_DIRS}
      ${PDAL_JSONCPP_INCLUDE_DIR}
)

PDAL_ADD_PLUGIN(voxelgrid_filter_libname filter voxelgrid
    FILES
        filters/VoxelGridFilter.cpp
    LINK_WITH ${PCL_LIBRARIES})
target_include_directories(${voxelgrid_filter_libname} PRIVATE
    ${INCLUDE_DIRS})

PDAL_ADD_PLUGIN(movingleastsquares_filter_libname filter movingleastsquares
    FILES
        filters/MovingLeastSquaresFilter.cpp
    LINK_WITH ${PCL_LIBRARIES})
target_include_directories(${movingleastsquares_filter_libname} PRIVATE
    ${INCLUDE_DIRS})

PDAL_ADD_PLUGIN(gridprojection_filter_libname filter gridprojection
    FILES
        filters/GridProjectionFilter.cpp
    LINK_WITH ${PCL_LIBRARIES})
target_include_directories(${gridprojection_filter_libname} PRIVATE
    ${INCLUDE_DIRS})

PDAL_ADD_PLUGIN(icp_filter_libname filter icp
    FILES
        filters/IcpFilter.cpp
    LINK_WITH ${PCL_LIBRARIES})
target_include_directories(${icp_filter_libname} PRIVATE
    ${INCLUDE_DIRS})

#
# PCL Kernel
#
PDAL_ADD_PLUGIN(pcl_libname kernel pcl
    FILES
        kernel/PCLKernel.cpp
    LINK_WITH ${PCL_LIBRARIES} ${pclblock_libname})
target_include_directories(${pcl_libname} PRIVATE
    ${PDAL_IO_DIR}
    ${INCLUDE_DIRS})

#
# Smooth Kernel
#

PDAL_ADD_PLUGIN(smooth_libname kernel smooth
    FILES
        kernel/SmoothKernel.cpp
    LINK_WITH ${PCL_LIBRARIES} ${pclblock_libname})
target_include_directories(${smooth_libname} PRIVATE
    ${INCLUDE_DIRS}
    ${PDAL_IO_DIR})

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_filters_pcl_block_test
    FILES
        test/PCLBlockFilterTest.cpp
    LINK_WITH ${pcd_reader_libname} ${pcd_writer_libname} ${pclblock_libname}
        ${pcl_libname} ${smooth_libname}
    )

    PDAL_ADD_TEST(pdal_filters_icp_test
    FILES
        test/IcpFilterTest.cpp
    LINK_WITH
        ${icp_filter_libname}
    )
    target_include_directories(pdal_filters_icp_test PRIVATE ${INCLUDE_DIRS})
endif()
