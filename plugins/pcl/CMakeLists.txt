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

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugin/pcl)

find_package(PCL 1.7 REQUIRED)
if(PCL_FOUND)
    message(STATUS "Building with PCL support")

    set(PCL_LINKAGE "LINK_PUBLIC;general")
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    add_definitions("-fvisibility-inlines-hidden")

    # PCL's configuration clobbers Boost find_package - do it again
    find_package(Boost 1.52 COMPONENTS program_options thread iostreams filesystem system unit_test_framework random)

    if (PCL_VISUALIZATION_FOUND)
        message(STATUS "Building with PCL Visualization support")
	#
	# PCLVisualizer Writer
	#
	set(srcs
	    drivers/PCLVisualizer.cpp
	)

	set(incs
	    drivers/PCLVisualizer.hpp
	    PCLConversions.hpp
	)

	set(PCLVIZ_LIB_NAME pdal_plugin_writer_pclvisualizer)
	PDAL_ADD_PLUGIN(${PCLVIZ_LIB_NAME} writer pclvisualizer ${srcs} ${incs})
    else()
        message(STATUS "..building without PCL Visualization")
    endif()

    #
    # PCD Reader
    #
    set(srcs
	drivers/PcdCommon.cpp
	drivers/PcdReader.cpp
    )

    set(incs
	drivers/PcdCommon.hpp
	drivers/PcdReader.hpp
	drivers/point_types.hpp
	PCLConversions.hpp
    )

    set(PCD_READ_LIB_NAME pdal_plugin_reader_pcd)
    PDAL_ADD_PLUGIN(${PCD_READ_LIB_NAME} reader pcd ${srcs} ${incs})
    target_link_libraries(${PCD_READ_LIB_NAME} ${PDAL_LINKAGE} ${PCL_LIBRARIES})

    #
    # PCD Writer
    #
    set(srcs
	drivers/PcdCommon.cpp
	drivers/PcdWriter.cpp
    )

    set(incs
	drivers/PcdCommon.hpp
	drivers/PcdWriter.hpp
	drivers/point_types.hpp
	PCLConversions.hpp
    )

    set(PCD_WRITE_LIB_NAME pdal_plugin_writer_pcd)
    PDAL_ADD_PLUGIN(${PCD_WRITE_LIB_NAME} writer pcd ${srcs} ${incs})
    target_link_libraries(${PCD_WRITE_LIB_NAME} ${PDAL_LINKAGE} ${PCL_LIBRARIES})

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

    set(PCLBLOCK_LIB_NAME pdal_plugin_filter_pclblock)
    PDAL_ADD_PLUGIN(${PCLBLOCK_LIB_NAME} filter pclblock ${srcs} ${incs})
    target_link_libraries(${PCLBLOCK_LIB_NAME} ${PDAL_LINKAGE} ${PCL_LIBRARIES})
    #
    # PCL Kernel
    #
    set(srcs
	kernel/PCL.cpp
    )

    set(incs
	kernel/PCL.hpp
	#PCLConversions.hpp
    )

    set(PCL_KERNEL_LIB_NAME pdal_plugin_kernel_pcl)
    PDAL_ADD_PLUGIN(${PCL_KERNEL_LIB_NAME} kernel pcl ${srcs} ${incs})
    target_link_libraries(${PCL_KERNEL_LIB_NAME} ${PCLBLOCK_LIB_NAME})

    #
    # Ground Kernel
    #
    set(srcs
	kernel/Ground.cpp
    )

    set(incs
	kernel/Ground.hpp
    )

    set(GROUND_KERNEL_LIB_NAME pdal_plugin_kernel_ground)
    PDAL_ADD_PLUGIN(${GROUND_KERNEL_LIB_NAME} kernel ground ${srcs} ${incs})
    target_link_libraries(${GROUND_KERNEL_LIB_NAME} ${PCLBLOCK_LIB_NAME})

    #
    # Smooth Kernel
    #
    set(srcs
	kernel/Smooth.cpp
    )

    set(incs
	kernel/Smooth.hpp
    )

    set(SMOOTH_KERNEL_LIB_NAME pdal_plugin_kernel_smooth)
    PDAL_ADD_PLUGIN(${SMOOTH_KERNEL_LIB_NAME} kernel smooth ${srcs} ${incs})
    target_link_libraries(${SMOOTH_KERNEL_LIB_NAME} ${PCLBLOCK_LIB_NAME})

    #
    # View Kernel
    #
    set(srcs
	kernel/View.cpp
    )

    set(incs
	kernel/View.hpp
	#PCLConversions.hpp
    )

    set(VIEW_KERNEL_LIB_NAME pdal_plugin_kernel_view)
    PDAL_ADD_PLUGIN(${VIEW_KERNEL_LIB_NAME} kernel view ${srcs} ${incs})
else()
    message(STATUS "Building without PCL support")
endif()
