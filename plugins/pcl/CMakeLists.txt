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

add_definitions(${PCL_DEFINITIONS})
#
# This can cause problems if you inline some code that instantiates a static.
# I'm not sure why it was here, but I'm keeping it for now in case someone
# goes looking for it. - ABELL 3/18
#if (NOT WIN32)
#    add_definitions("-fvisibility-inlines-hidden")
#endif()

if (WIN32)
    add_definitions("-DNOMINMAX")
endif()

# PCL's configuration clobbers Boost find_package - do it again
#find_package(Boost QUIET 1.52 COMPONENTS program_options iostreams filesystem system thread)

#set (PCL_LIBRARIES "${PCL_COMMON_LIBRARY};${PCL_REGISTRATION_LIBRARY};${PCL_FEATURES_LIBRARY};${PCL_FILTERS_LIBRARY};${PCL_SEARCH_LIBRARY};${PCL_SEGMENTATION_LIBRARY};${PCL_IO_LIBRARY}")
#
# PCD Reader
#
PDAL_ADD_PLUGIN(pcd_reader_libname reader pcd
    FILES
        io/PcdHeader.hpp
        io/PcdHeader.cpp
        io/PcdReader.hpp
        io/PcdReader.cpp
    LINK_WITH
        ${PCL_LIBRARIES}
    INCLUDES
        ${PCL_INCLUDE_DIRS}
)
    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU" )
        set_source_files_properties(io/PcdReader.cpp PROPERTIES COMPILE_FLAGS -Wno-pedantic)
    endif()

#
# PCD Writer
#
PDAL_ADD_PLUGIN(pcd_writer_libname writer pcd
    FILES
        io/PcdWriter.cpp
    LINK_WITH
        ${PCL_LIBRARIES}
    INCLUDES
        ${PCL_INCLUDE_DIRS}
    )
    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU" )
        set_source_files_properties(io/PcdWriter.cpp PROPERTIES COMPILE_FLAGS -Wno-pedantic)
    endif()

#
# PCLBlock Filter
#
PDAL_ADD_PLUGIN(pclblock_libname filter pclblock
    FILES
        pipeline/PCLPipeline.cpp
        filters/PCLBlock.cpp
    LINK_WITH
        ${PCL_LIBRARIES}
    INCLUDES
        ${PCL_INCLUDE_DIRS}
        ${NLOHMANN_INCLUDE_DIR}
)

PDAL_ADD_PLUGIN(voxelgrid_filter_libname filter voxelgrid
    FILES
        filters/VoxelGridFilter.cpp
    LINK_WITH
        ${PCL_LIBRARIES}
    INCLUDES
        ${PCL_INCLUDE_DIRS}
)

PDAL_ADD_PLUGIN(movingleastsquares_filter_libname filter movingleastsquares
    FILES
        filters/MovingLeastSquaresFilter.cpp
    LINK_WITH
        ${PCL_LIBRARIES}
    INCLUDES
        ${PCL_INCLUDE_DIRS}
)

PDAL_ADD_PLUGIN(gridprojection_filter_libname filter gridprojection
    FILES
        filters/GridProjectionFilter.cpp
    LINK_WITH
        ${PCL_LIBRARIES}
    INCLUDES
        ${PCL_INCLUDE_DIRS}
)

#
# PCL Kernel
#
PDAL_ADD_PLUGIN(pcl_libname kernel pcl
    FILES
        kernel/PCLKernel.cpp
    LINK_WITH
        ${PCL_LIBRARIES}
        ${pclblock_libname}
    INCLUDES
        ${PDAL_IO_DIR}
        ${PCL_INCLUDE_DIRS}
        ${PDAL_VENDOR_DIR}/eigen
)

#
# Smooth Kernel
#
PDAL_ADD_PLUGIN(smooth_libname kernel smooth
    FILES
        kernel/SmoothKernel.cpp
    LINK_WITH
        ${PCL_LIBRARIES}
        ${pclblock_libname}
    INCLUDES
        ${PCL_INCLUDE_DIRS}
        ${PDAL_IO_DIR}
)

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_filters_pcl_block_test
    FILES
        test/PCLBlockFilterTest.cpp
    LINK_WITH ${pcd_reader_libname} ${pcd_writer_libname} ${pclblock_libname}
        ${pcl_libname} ${smooth_libname}
    )
endif()
