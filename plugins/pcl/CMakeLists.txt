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
include_directories(${PROJECT_SOURCE_DIR}/plugins/pcl)
include_directories(${PROJECT_SOURCE_DIR}/io/las)
include_directories(${PROJECT_SOURCE_DIR}/kernels)

find_package(PCL 1.7)
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
	    io/PCLVisualizer.cpp
	)

	set(incs
	    io/PCLVisualizer.hpp
	    PCLConversions.hpp
	)

        set(deps ${PCL_LIBRARIES})
        PDAL_ADD_PLUGIN(pclvisualizer_writer_lib_name writer pclvisualizer "${srcs}" "${incs}" "${deps}")

	#
        # View Kernel
        #
        set(srcs
            kernel/ViewKernel.cpp
        )

        set(incs
            kernel/ViewKernel.hpp
        )

        set(deps ${PCL_LIBRARIES})
        PDAL_ADD_PLUGIN(view_kernel_lib_name kernel view "${srcs}" "${incs}" "${deps}")
    else()
        message(STATUS "..building without PCL Visualization")
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

    set(deps ${PCL_LIBRARIES})
    PDAL_ADD_PLUGIN(pcd_reader_lib_name reader pcd "${srcs}" "${incs}" "${deps}")

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

    set(deps ${PCL_LIBRARIES})
    PDAL_ADD_PLUGIN(pcd_writer_lib_name writer pcd "${srcs}" "${incs}" "${deps}")

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

    set(deps ${PCL_LIBRARIES})
    PDAL_ADD_PLUGIN(pclblock_filter_lib_name filter pclblock "${srcs}" "${incs}" "${deps}")

    #
    # PCL Kernel
    #
    set(srcs
        kernel/PCLKernel.cpp
    )

    set(incs
        kernel/PCLKernel.hpp
    )

    set(deps ${PCL_LIBRARIES} ${pclblock_filter_lib_name})
    PDAL_ADD_PLUGIN(pcl_kernel_lib_name kernel pcl "${srcs}" "${incs}" "${deps}")

    #
    # Ground Kernel
    #
    set(srcs
        kernel/GroundKernel.cpp
    )

    set(incs
        kernel/GroundKernel.hpp
    )

    set(deps ${PCL_LIBRARIES} ${pclblock_filter_lib_name})
    PDAL_ADD_PLUGIN(ground_kernel_lib_name kernel ground "${srcs}" "${incs}" "${deps}")

    #
    # Smooth Kernel
    #
    set(srcs
        kernel/SmoothKernel.cpp
    )

    set(incs
        kernel/SmoothKernel.hpp
    )

    set(deps ${PCL_LIBRARIES} ${pclblock_filter_lib_name})
    PDAL_ADD_PLUGIN(smooth_kernel_lib_name kernel smooth "${srcs}" "${incs}" "${deps}")

    if (WITH_TESTS)
        #
        # PCL tests
        #
	set(srcs
	    test/PCLBlockFilterTest.cpp
	)

	set(deps
	    ${pclvisualizer_lib_name}
	    ${pcd_reader_lib_name}
	    ${pcd_writer_lib_name}
	    ${pclblock_filter_lib_name}
	    ${pcl_kernel_lib_name}
	    ${ground_kernel_lib_name}
	    ${smooth_kernel_lib_name}
	)
	PDAL_ADD_TEST(pcltest "${srcs}" "${deps}")
    endif()
else()
    message(STATUS "Building without PCL support")
endif()
