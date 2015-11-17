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

set_package_properties(PCL PROPERTIES DESCRIPTION "Point Cloud Library"
    URL "http://pointclouds.org" TYPE RECOMMENDED
    PURPOSE "Enables PCD reader/writer, PCLVisualizer, PCLBlock filter, and ground, pcl, smooth, and view kernels")

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_LIST_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/filters)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/pipeline)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/dartsample)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
if (NOT WIN32)
    add_definitions("-fvisibility-inlines-hidden")
endif()

# PCL's configuration clobbers Boost find_package - do it again
find_package(Boost QUIET 1.52 COMPONENTS program_options iostreams filesystem system thread)

if (PCL_VISUALIZATION_FOUND)
#
# PCLVisualizer Writer
#
    set(srcs io/PCLVisualizer.cpp)
    set(incs
        io/PCLVisualizer.hpp
        PCLConversions.hpp
    )

    PDAL_ADD_PLUGIN(pclvisualizer_libname writer pclvisualizer
        FILES "${srcs}" "${incs}"
        LINK_WITH ${PCL_LIBRARIES})

    #
    # View Kernel
    #
    set(srcs kernel/ViewKernel.cpp)
    set(incs kernel/ViewKernel.hpp)

    PDAL_ADD_PLUGIN(view_libname kernel view
        FILES "${srcs}" "${incs}"
        LINK_WITH ${PCL_LIBRARIES})
else()
    message(STATUS "PCLVisualizer disabled because PCL_VISUALIZATION was not found")
endif()

#
# PCD Reader
#
set(srcs
    io/PcdCommon.cpp
    io/PcdReader.cpp
)

set(incs
    io/PcdCommon.hpp
    io/PcdReader.hpp
    io/point_types.hpp
    PCLConversions.hpp
)

PDAL_ADD_PLUGIN(pcd_reader_libname reader pcd
    FILES "${srcs}" "${incs}"
    LINK_WITH ${PCL_LIBRARIES})

#
# PCD Writer
#
set(srcs
    io/PcdCommon.cpp
    io/PcdWriter.cpp
)

set(incs
    io/PcdCommon.hpp
    io/PcdWriter.hpp
    io/point_types.hpp
    PCLConversions.hpp
)

PDAL_ADD_PLUGIN(pcd_writer_libname writer pcd
    FILES "${srcs}" "${incs}"
    LINK_WITH ${PCL_LIBRARIES})

#
# PCLBlock Filter
#
set(srcs
    pipeline/PCLPipeline.cpp
    filters/PCLBlock.cpp
)

set(incs
   pipeline/PCLPipeline.h
   pipeline/PCLPipeline.hpp
   filters/PCLBlock.hpp
   PCLConversions.hpp
)

PDAL_ADD_PLUGIN(pclblock_libname filter pclblock
    FILES "${srcs}" "${incs}"
    LINK_WITH ${PCL_LIBRARIES})

#
# Ground Filter
#
PDAL_ADD_PLUGIN(ground_filter_libname filter ground
    FILES filters/GroundFilter.hpp filters/GroundFilter.cpp
    LINK_WITH ${PCL_LIBRARIES})

PDAL_ADD_PLUGIN(statistical_outlier_filter_libname filter statisticaloutlier
    FILES filters/StatisticalOutlierFilter.hpp filters/StatisticalOutlierFilter.cpp
    LINK_WITH ${PCL_LIBRARIES})

PDAL_ADD_PLUGIN(radius_outlier_filter_libname filter radiusoutlier
    FILES filters/RadiusOutlierFilter.hpp filters/RadiusOutlierFilter.cpp
    LINK_WITH ${PCL_LIBRARIES})

PDAL_ADD_PLUGIN(voxelgrid_filter_libname filter voxelgrid
    FILES filters/VoxelGridFilter.hpp filters/VoxelGridFilter.cpp
    LINK_WITH ${PCL_LIBRARIES})

PDAL_ADD_PLUGIN(movingleastsquares_filter_libname filter movingleastsquares
    FILES filters/MovingLeastSquaresFilter.hpp filters/MovingLeastSquaresFilter.cpp
    LINK_WITH ${PCL_LIBRARIES})

PDAL_ADD_PLUGIN(gridprojection_filter_libname filter gridprojection
    FILES filters/GridProjectionFilter.hpp filters/GridProjectionFilter.cpp
    LINK_WITH ${PCL_LIBRARIES})

PDAL_ADD_PLUGIN(poisson_filter_libname filter poisson
    FILES filters/PoissonFilter.hpp filters/PoissonFilter.cpp
    LINK_WITH ${PCL_LIBRARIES})

PDAL_ADD_PLUGIN(greedyprojection_filter_libname filter greedyprojection
    FILES filters/GreedyProjectionFilter.hpp filters/GreedyProjectionFilter.cpp
    LINK_WITH ${PCL_LIBRARIES})

PDAL_ADD_PLUGIN(height_filter_libname filter height
    FILES filters/HeightFilter.cpp
    LINK_WITH ${PCL_LIBRARIES})

PDAL_ADD_PLUGIN(dartsample_filter_libname filter dartsample
    FILES
        filters/DartSampleFilter.cpp
        dartsample/dart_sample.cpp
    LINK_WITH ${PCL_LIBRARIES})

#
# PCL Kernel
#
set(srcs kernel/PCLKernel.cpp)
set(incs kernel/PCLKernel.hpp)

PDAL_ADD_PLUGIN(pcl_libname kernel pcl
    FILES "${srcs}" "${incs}"
    LINK_WITH ${PCL_LIBRARIES} ${pclblock_libname})

#
# Height Above Ground Kernel
#
#PDAL_ADD_PLUGIN(hag_libname kernel height
#    FILES kernel/HeightAboveGroundKernel.cpp
#    LINK_WITH ${PCL_LIBRARIES})

#
# Ground Kernel
#
set(srcs kernel/GroundKernel.cpp)
set(incs kernel/GroundKernel.hpp)

PDAL_ADD_PLUGIN(ground_kernel_libname kernel ground
    FILES "${srcs}" "${incs}"
    LINK_WITH ${ground_filter_libname})

#
# Smooth Kernel
#
set(srcs kernel/SmoothKernel.cpp)
set(incs kernel/SmoothKernel.hpp)

PDAL_ADD_PLUGIN(smooth_libname kernel smooth
    FILES "${srcs}" "${incs}"
    LINK_WITH ${PCL_LIBRARIES} ${pclblock_libname})

if (WITH_TESTS)
    PDAL_ADD_TEST(pcltest
    FILES test/PCLBlockFilterTest.cpp
    LINK_WITH ${pclvisualizer_libname} ${pcd_reader_libname}
        ${pcd_writer_libname} ${pclblock_libname} ${ground_filter_libname}
        ${pcl_libname} ${ground_kernel_libname} ${smooth_libname}
    )
endif()
