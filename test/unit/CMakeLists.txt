###############################################################################
#
# test/unit/CMakeLists.txt controls building of PDAL unit tests suite
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
###############################################################################

configure_file(TestConfig.hpp.in "${CMAKE_CURRENT_BINARY_DIR}/TestConfig.hpp")

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${GDAL_INCLUDE_DIR}
    ${GEOTIFF_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/io/bpf
    ${PROJECT_SOURCE_DIR}/io/buffer
    ${PROJECT_SOURCE_DIR}/io/faux
    ${PROJECT_SOURCE_DIR}/io/las
    ${PROJECT_SOURCE_DIR}/io/qfit
    ${PROJECT_SOURCE_DIR}/io/sbet
    ${PROJECT_SOURCE_DIR}/io/text
    ${PROJECT_SOURCE_DIR}/io/terrasolid
    ${PROJECT_SOURCE_DIR}/filters/chipper
    ${PROJECT_SOURCE_DIR}/filters/colorization
    ${PROJECT_SOURCE_DIR}/filters/crop
    ${PROJECT_SOURCE_DIR}/filters/decimation
    ${PROJECT_SOURCE_DIR}/filters/ferry
    ${PROJECT_SOURCE_DIR}/filters/mortonorder
    ${PROJECT_SOURCE_DIR}/filters/reprojection
    ${PROJECT_SOURCE_DIR}/filters/sort
    ${PROJECT_SOURCE_DIR}/filters/splitter
    ${PROJECT_SOURCE_DIR}/filters/stats
    ${PROJECT_SOURCE_DIR}/kernels/info
)

PDAL_ADD_TEST(pdal_bounds_test BoundsTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_config_test ConfigTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_environment_test EnvironmentTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_file_utils_test FileUtilsTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_gdal_utils_test GDALUtilsTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_log_test LogTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_metadata_test MetadataTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_options_test OptionsTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_pipeline_manager_test PipelineManagerTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_point_buffer_test PointBufferTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_point_context_test PointContextTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_spatial_reference_test SpatialReferenceTest.cpp ${PDAL_LIB_NAME})
#PDAL_ADD_TEST(pdal_stage_factory_test StageFactoryTest.cpp ${PDAL_LIB_NAME}) # this one has been disabled for a while, can it go?
PDAL_ADD_TEST(pdal_stream_factory_test StreamFactoryTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_support_test SupportTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_user_callback_test UserCallbackTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_utils_test UtilsTest.cpp ${PDAL_LIB_NAME})

if (PDAL_HAVE_LAZPERF)
    PDAL_ADD_TEST(pdal_lazperf_test CompressionTest.cpp ${PDAL_LIB_NAME})
endif()

#
# sources for the native io
#
PDAL_ADD_TEST(pdal_io_bpf_test io/bpf/BPFTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_io_buffer_test io/buffer/BufferTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_io_faux_test io/faux/FauxReaderTest.cpp ${PDAL_LIB_NAME})
set(srcs
    io/las/LasReaderTest.cpp
    io/las/LasWriterTest.cpp
)
PDAL_ADD_TEST(pdal_io_las_test "${srcs}" ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_io_qfit_test io/qfit/QFITReaderTest.cpp ${PDAL_LIB_NAME})
set(srcs
    io/sbet/SbetReaderTest.cpp
    io/sbet/SbetWriterTest.cpp
)
PDAL_ADD_TEST(pdal_io_sbet_test "${srcs}" ${PDAL_LIB_NAME})
#PDAL_ADD_TEST(pdal_io_terrasolid_test io/terrasolid/TerraSolidReaderTest.cpp ${PDAL_LIB_NAME})
#PDAL_ADD_TEST(pdal_io_text_test io/text/TextWriterTest.cpp ${PDAL_LIB_NAME})

#
# sources for the native filters
#
PDAL_ADD_TEST(pdal_filters_chipper_test filters/ChipperTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_filters_colorization_test filters/ColorizationFilterTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_filters_crop_test filters/CropFilterTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_filters_decimation_test filters/DecimationFilterTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_filters_ferry_test filters/FerryFilterTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_filters_merge_test filters/MergeTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_filters_reprojection_test filters/ReprojectionFilterTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_filters_sort_test filters/SortFilterTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_filters_splitter_test filters/SplitterTest.cpp ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_filters_stats_test filters/StatsFilterTest.cpp ${PDAL_LIB_NAME})

#
# sources for plang
#
if (PDAL_HAVE_PYTHON)
    PDAL_ADD_TEST(pdal_plang_test plang/PLangTest.cpp ${PDAL_LIB_NAME})
endif()

#
# conditionally append apps/libxml2 sources
#
if(WITH_APPS)
#    if(LIBXML2_FOUND)
#        PDAL_ADD_TEST(pcinfo_test apps/pcinfoTest.cpp ${PDAL_LIB_NAME})
#    endif(LIBXML2_FOUND)

    PDAL_ADD_TEST(pc2pc_test apps/pc2pcTest.cpp ${PDAL_LIB_NAME})
    #PDAL_ADD_TEST(pcpipeline_test apps/pcpipelineTest.cpp ${PDAL_LIB_NAME})
endif(WITH_APPS)

if(LIBXML2_FOUND)
    PDAL_ADD_TEST(xml_schema_test XMLSchemaTest.cpp ${PDAL_LIB_NAME})
endif()
