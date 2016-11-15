###############################################################################
#
# test/unit/CMakeLists.txt controls building of PDAL unit tests suite
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
###############################################################################

configure_file(TestConfig.hpp.in "${CMAKE_CURRENT_BINARY_DIR}/TestConfig.hpp")

file(GLOB_RECURSE inFiles RELATIVE "${CMAKE_SOURCE_DIR}" "${CMAKE_SOURCE_DIR}/test/data/*.in")
foreach(infileName ${inFiles})
    string(REGEX REPLACE ".in\$" "" outfileName "${infileName}")
    configure_file("${CMAKE_SOURCE_DIR}/${infileName}"
        "${CMAKE_BINARY_DIR}/${outfileName}")
endforeach()

PDAL_ADD_TEST(pdal_bounds_test FILES BoundsTest.cpp)
PDAL_ADD_TEST(pdal_config_test FILES ConfigTest.cpp)
PDAL_ADD_TEST(pdal_eigen_test FILES EigenTest.cpp)
PDAL_ADD_TEST(pdal_file_utils_test FILES FileUtilsTest.cpp)
PDAL_ADD_TEST(pdal_georeference_test FILES GeoreferenceTest.cpp)
PDAL_ADD_TEST(pdal_kdindex_test FILES KDIndexTest.cpp)
PDAL_ADD_TEST(pdal_kernel_test FILES KernelTest.cpp)
PDAL_ADD_TEST(pdal_log_test FILES LogTest.cpp)
target_include_directories(pdal_log_test PRIVATE
    ${PROJECT_SOURCE_DIR}/io/faux)
PDAL_ADD_TEST(pdal_metadata_test FILES MetadataTest.cpp)
PDAL_ADD_TEST(pdal_options_test FILES OptionsTest.cpp)
target_include_directories(pdal_options_test PRIVATE
    ${PROJECT_SOURCE_DIR}/filters/crop
    ${PROJECT_SOURCE_DIR}/io/faux)
PDAL_ADD_TEST(pdal_pipeline_manager_test FILES PipelineManagerTest.cpp)
PDAL_ADD_TEST(pdal_plugin_manager_test FILES PluginManagerTest.cpp)
PDAL_ADD_TEST(pdal_point_view_test FILES PointViewTest.cpp)
PDAL_ADD_TEST(pdal_point_table_test FILES PointTableTest.cpp)
target_include_directories(pdal_point_table_test PRIVATE
    ${PDAL_IO_DIR})
PDAL_ADD_TEST(pdal_program_arg_test FILES ProgramArgsTest.cpp)
PDAL_ADD_TEST(pdal_polygon_test FILES PolygonTest.cpp)
PDAL_ADD_TEST(pdal_spatial_reference_test FILES SpatialReferenceTest.cpp)
target_include_directories(pdal_spatial_reference_test PRIVATE
    ${PROJECT_SOURCE_DIR}/filters/merge
    ${PROJECT_SOURCE_DIR}/filters/reprojection
    ${PROJECT_SOURCE_DIR}/io/las)
PDAL_ADD_TEST(pdal_stage_factory_test FILES StageFactoryTest.cpp)
PDAL_ADD_TEST(pdal_streaming_test FILES StreamingTest.cpp)
target_include_directories(pdal_streaming_test PRIVATE
    ${PROJECT_SOURCE_DIR}/filters/streamcallback
    ${PROJECT_SOURCE_DIR}/filters/merge
    ${PROJECT_SOURCE_DIR}/io/faux)
PDAL_ADD_TEST(pdal_support_test FILES SupportTest.cpp)
PDAL_ADD_TEST(pdal_utils_test FILES UtilsTest.cpp)
PDAL_ADD_TEST(pdal_uuid_test FILES UuidTest.cpp)
if (PDAL_HAVE_LAZPERF)
    PDAL_ADD_TEST(pdal_lazperf_test FILES CompressionTest.cpp)
    target_include_directories(pdal_lazperf_test PRIVATE
        ${PROJECT_SOURCE_DIR}/io/las)
endif()

#
# sources for the native io
#
PDAL_ADD_TEST(pdal_io_bpf_test FILES io/bpf/BPFTest.cpp)
target_include_directories(pdal_io_bpf_test PRIVATE
    ${PROJECT_SOURCE_DIR}/io/buffer
    ${PROJECT_SOURCE_DIR}/io/bpf)
PDAL_ADD_TEST(pdal_io_buffer_test FILES io/buffer/BufferTest.cpp)
target_include_directories(pdal_io_buffer_test PRIVATE
    ${PROJECT_SOURCE_DIR}/filters/stats
    ${PROJECT_SOURCE_DIR}/io/buffer)
PDAL_ADD_TEST(pdal_io_faux_test FILES io/faux/FauxReaderTest.cpp)
target_include_directories(pdal_io_faux_test PRIVATE
    ${PROJECT_SOURCE_DIR}/io/faux)
PDAL_ADD_TEST(pdal_io_gdal_reader_test FILES io/gdal/GDALReaderTest.cpp)
target_include_directories(pdal_io_gdal_reader_test PRIVATE
    ${PROJECT_SOURCE_DIR}/io/gdal)
PDAL_ADD_TEST(pdal_io_gdal_writer_test FILES io/gdal/GDALWriterTest.cpp)
target_include_directories(pdal_io_gdal_writer_test PRIVATE
    ${PROJECT_SOURCE_DIR}/io/las
    ${PROJECT_SOURCE_DIR}/io/text
    ${PROJECT_SOURCE_DIR}/io/gdal)
PDAL_ADD_TEST(pdal_io_ilvis2_test FILES io/ilvis2/Ilvis2ReaderTest.cpp)
target_include_directories(pdal_io_ilvis2_test PRIVATE
    ${PROJECT_SOURCE_DIR}/io/ilvis2)
PDAL_ADD_TEST(pdal_io_las_reader_test FILES io/las/LasReaderTest.cpp)
target_include_directories(pdal_io_las_reader_test PRIVATE
    ${PROJECT_SOURCE_DIR}/io/las)
PDAL_ADD_TEST(pdal_io_las_writer_test FILES io/las/LasWriterTest.cpp)
target_include_directories(pdal_io_las_writer_test PRIVATE
    ${PROJECT_SOURCE_DIR}/io/bpf
    ${PROJECT_SOURCE_DIR}/io/buffer
    ${PROJECT_SOURCE_DIR}/io/las)
PDAL_ADD_TEST(pdal_io_optech_test FILES io/optech/OptechReaderTest.cpp)
target_include_directories(pdal_io_optech_test PRIVATE
    ${PROJECT_SOURCE_DIR}/io/optech)
PDAL_ADD_TEST(pdal_io_ply_reader_test FILES io/ply/PlyReaderTest.cpp)
target_include_directories(pdal_io_ply_reader_test PRIVATE
    ${PROJECT_SOURCE_DIR}/io/ply)
PDAL_ADD_TEST(pdal_io_ply_writer_test FILES io/ply/PlyWriterTest.cpp)
target_include_directories(pdal_io_ply_writer_test PRIVATE
    ${PROJECT_SOURCE_DIR}/io/faux
    ${PROJECT_SOURCE_DIR}/io/ply)
PDAL_ADD_TEST(pdal_io_pts_reader_test FILES io/pts/PtsReaderTest.cpp)
target_include_directories(pdal_io_pts_reader_test PRIVATE
    ${PROJECT_SOURCE_DIR}/io/pts)
PDAL_ADD_TEST(pdal_io_qfit_test FILES io/qfit/QFITReaderTest.cpp)
target_include_directories(pdal_io_qfit_test PRIVATE
    ${PROJECT_SOURCE_DIR}/io/qfit)
PDAL_ADD_TEST(pdal_io_sbet_reader_test FILES io/sbet/SbetReaderTest.cpp)
target_include_directories(pdal_io_sbet_reader_test PRIVATE
    ${PROJECT_SOURCE_DIR}/io/sbet)
PDAL_ADD_TEST(pdal_io_sbet_writer_test FILES io/sbet/SbetWriterTest.cpp)
target_include_directories(pdal_io_sbet_writer_test PRIVATE
    ${PROJECT_SOURCE_DIR}/io/sbet)
PDAL_ADD_TEST(pdal_io_terrasolid_test FILES
    io/terrasolid/TerrasolidReaderTest.cpp)
target_include_directories(pdal_io_terrasolid_test PRIVATE
    ${PROJECT_SOURCE_DIR}/io/terrasolid)
PDAL_ADD_TEST(pdal_io_text_reader_test FILES io/text/TextReaderTest.cpp)
target_include_directories(pdal_io_text_reader_test PRIVATE
    ${PROJECT_SOURCE_DIR}/io/las
    ${PROJECT_SOURCE_DIR}/io/text)
PDAL_ADD_TEST(pdal_io_text_writer_test FILES io/text/TextWriterTest.cpp)
target_include_directories(pdal_io_text_writer_test PRIVATE
    ${PROJECT_SOURCE_DIR}/io/text)

#
# sources for the native filters
#
PDAL_ADD_TEST(pdal_filters_attribute_test FILES filters/AttributeFilterTest.cpp)
PDAL_ADD_TEST(pdal_filters_chipper_test FILES filters/ChipperTest.cpp)
target_include_directories(pdal_filters_chipper_test PRIVATE
    ${PROJECT_SOURCE_DIR}/filters/chipper
    ${PROJECT_SOURCE_DIR}/io/las )
PDAL_ADD_TEST(pdal_filters_colorization_test FILES
    filters/ColorizationFilterTest.cpp)
target_include_directories(pdal_filters_colorization_test PRIVATE
    ${PROJECT_SOURCE_DIR}/filters/streamcallback
    ${PROJECT_SOURCE_DIR}/filters/colorization
    ${PROJECT_SOURCE_DIR}/io/las )
PDAL_ADD_TEST(pdal_filters_computerange_test FILES filters/ComputeRangeFilterTest.cpp)
PDAL_ADD_TEST(pdal_filters_crop_test FILES filters/CropFilterTest.cpp)
target_include_directories(pdal_filters_crop_test PRIVATE
    ${PROJECT_SOURCE_DIR}/filters/crop
    ${PROJECT_SOURCE_DIR}/filters/reprojection
    ${PROJECT_SOURCE_DIR}/filters/stats
    ${PROJECT_SOURCE_DIR}/filters/streamcallback
    ${PROJECT_SOURCE_DIR}/io/faux
    ${PROJECT_SOURCE_DIR}/io/las
    ${PROJECT_SOURCE_DIR}/io/buffer)
PDAL_ADD_TEST(pdal_filters_decimation_test FILES
    filters/DecimationFilterTest.cpp)
target_include_directories(pdal_filters_decimation_test PRIVATE
    ${PROJECT_SOURCE_DIR}/filters/streamcallback
    ${PROJECT_SOURCE_DIR}/filters/decimation
    ${PROJECT_SOURCE_DIR}/io/faux)
PDAL_ADD_TEST(pdal_filters_divider_test FILES filters/DividerFilterTest.cpp)
target_include_directories(pdal_filters_divider_test PRIVATE
    ${PROJECT_SOURCE_DIR}/filters/divider
    ${PROJECT_SOURCE_DIR}/io/faux)
PDAL_ADD_TEST(pdal_filters_ferry_test FILES filters/FerryFilterTest.cpp)
target_include_directories(pdal_filters_ferry_test PRIVATE
    ${PROJECT_SOURCE_DIR}/filters/streamcallback
    ${PROJECT_SOURCE_DIR}/filters/ferry
    ${PROJECT_SOURCE_DIR}/io/las
    ${PROJECT_SOURCE_DIR}/io/faux)
PDAL_ADD_TEST(pdal_filters_merge_test FILES filters/MergeTest.cpp)
PDAL_ADD_TEST(pdal_filters_additional_merge_test FILES
    filters/AdditionalMergeTest.cpp)
target_include_directories(pdal_filters_additional_merge_test PRIVATE
    ${PROJECT_SOURCE_DIR}/filters/merge
    ${PROJECT_SOURCE_DIR}/filters/decimation
    ${PROJECT_SOURCE_DIR}/io/las)
PDAL_ADD_TEST(pdal_filters_reprojection_test FILES
    filters/ReprojectionFilterTest.cpp)
target_include_directories(pdal_filters_reprojection_test PRIVATE
    ${PROJECT_SOURCE_DIR}/filters/streamcallback
    ${PROJECT_SOURCE_DIR}/filters/reprojection
    ${PROJECT_SOURCE_DIR}/io/las)
PDAL_ADD_TEST(pdal_filters_range_test FILES filters/RangeFilterTest.cpp)
target_include_directories(pdal_filters_range_test PRIVATE
    ${PROJECT_SOURCE_DIR}/filters/streamcallback
    ${PROJECT_SOURCE_DIR}/filters/range
    ${PROJECT_SOURCE_DIR}/io/faux)
PDAL_ADD_TEST(pdal_filters_randomize_test FILES filters/RandomizeFilterTest.cpp)
target_include_directories(pdal_filters_randomize_test PRIVATE
    ${PROJECT_SOURCE_DIR}/filters/randomize
    ${PROJECT_SOURCE_DIR}/io/faux)
PDAL_ADD_TEST(pdal_filters_sort_test FILES filters/SortFilterTest.cpp)
target_include_directories(pdal_filters_sort_test PRIVATE
    ${PROJECT_SOURCE_DIR}/filters/sort
    ${PROJECT_SOURCE_DIR}/io/las)
PDAL_ADD_TEST(pdal_filters_splitter_test FILES filters/SplitterTest.cpp)
target_include_directories(pdal_filters_splitter_test PRIVATE
    ${PROJECT_SOURCE_DIR}/filters/splitter
    ${PROJECT_SOURCE_DIR}/io/las)
PDAL_ADD_TEST(pdal_filters_stats_test FILES filters/StatsFilterTest.cpp)
target_include_directories(pdal_filters_stats_test PRIVATE
    ${PROJECT_SOURCE_DIR}/filters/stats
    ${PROJECT_SOURCE_DIR}/io/faux)
PDAL_ADD_TEST(pdal_filters_transformation_test FILES
    filters/TransformationFilterTest.cpp)
target_include_directories(pdal_filters_transformation_test PRIVATE
    ${PROJECT_SOURCE_DIR}/filters/transformation
    ${PROJECT_SOURCE_DIR}/io/faux)

#
# conditionally append apps/libxml2 sources
#
if (WITH_APPS)
    PDAL_ADD_TEST(pdal_app_test FILES apps/AppTest.cpp)
    if (LASZIP_FOUND)
        PDAL_ADD_TEST(pdal_merge_test FILES apps/MergeTest.cpp)
    endif()
    PDAL_ADD_TEST(pc2pc_test FILES apps/pc2pcTest.cpp)
    target_include_directories(pc2pc_test PRIVATE
        ${PROJECT_SOURCE_DIR}/io/las)

    if (BUILD_PIPELINE_TESTS)
        PDAL_ADD_TEST(pcpipeline_test FILES apps/pcpipelineTest.cpp)
        PDAL_ADD_TEST(pcpipeline_test_json FILES apps/pcpipelineTestJSON.cpp)
    endif()
    PDAL_ADD_TEST(random_test FILES apps/RandomTest.cpp)
    target_include_directories(random_test PRIVATE
        ${PROJECT_SOURCE_DIR}/io/las)
    PDAL_ADD_TEST(translate_test FILES apps/TranslateTest.cpp)
endif(WITH_APPS)

if(LIBXML2_FOUND)
    PDAL_ADD_TEST(pdal_io_ilvis2_metadata_test FILES
        io/ilvis2/Ilvis2MetadataReaderTest.cpp)
    target_include_directories(pdal_io_ilvis2_metadata_test PRIVATE
        ${PROJECT_SOURCE_DIR}/io/ilvis2 )
    PDAL_ADD_TEST(pdal_io_ilvis2_reader_metadata_test FILES
        io/ilvis2/Ilvis2ReaderWithMDReaderTest.cpp)
    target_include_directories(pdal_io_ilvis2_reader_metadata_test PRIVATE
        ${PROJECT_SOURCE_DIR}/io/ilvis2 )
    PDAL_ADD_TEST(xml_schema_test FILES XMLSchemaTest.cpp)
endif()
