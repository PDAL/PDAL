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
    configure_file("${CMAKE_SOURCE_DIR}/${infileName}" "${CMAKE_BINARY_DIR}/${outfileName}")
endforeach()
include_directories(${CMAKE_CURRENT_BINARY_DIR})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${GDAL_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/io/bpf
    ${PROJECT_SOURCE_DIR}/io/buffer
    ${PROJECT_SOURCE_DIR}/io/faux
    ${PROJECT_SOURCE_DIR}/io/las
    ${PROJECT_SOURCE_DIR}/io/optech
    ${PROJECT_SOURCE_DIR}/io/ply
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
    ${PROJECT_SOURCE_DIR}/filters/range
    ${PROJECT_SOURCE_DIR}/filters/sort
    ${PROJECT_SOURCE_DIR}/filters/splitter
    ${PROJECT_SOURCE_DIR}/filters/stats
    ${PROJECT_SOURCE_DIR}/filters/transformation
    ${PROJECT_SOURCE_DIR}/kernels/info
)

if (WITH_GEOTIFF)
    include_directories(${GEOTIFF_INCLUDE_DIR})
endif()

PDAL_ADD_TEST(pdal_bounds_test FILES BoundsTest.cpp)
PDAL_ADD_TEST(pdal_config_test FILES ConfigTest.cpp)
PDAL_ADD_TEST(pdal_file_utils_test FILES FileUtilsTest.cpp)
PDAL_ADD_TEST(pdal_georeference_test FILES GeoreferenceTest.cpp)
PDAL_ADD_TEST(pdal_kdindex_test FILES KDIndexTest.cpp)
PDAL_ADD_TEST(pdal_log_test FILES LogTest.cpp)
PDAL_ADD_TEST(pdal_metadata_test FILES MetadataTest.cpp)
PDAL_ADD_TEST(pdal_options_test FILES OptionsTest.cpp)
PDAL_ADD_TEST(pdal_pdalutils_test FILES PDALUtilsTest.cpp)
PDAL_ADD_TEST(pdal_pipeline_manager_test FILES PipelineManagerTest.cpp)
PDAL_ADD_TEST(pdal_point_view_test FILES PointViewTest.cpp)
PDAL_ADD_TEST(pdal_point_table_test FILES PointTableTest.cpp)
PDAL_ADD_TEST(pdal_spatial_reference_test FILES SpatialReferenceTest.cpp)
PDAL_ADD_TEST(pdal_support_test FILES SupportTest.cpp)
PDAL_ADD_TEST(pdal_user_callback_test FILES UserCallbackTest.cpp)
PDAL_ADD_TEST(pdal_utils_test FILES UtilsTest.cpp)

if (PDAL_HAVE_LAZPERF)
    PDAL_ADD_TEST(pdal_lazperf_test FILES CompressionTest.cpp)
endif()

#
# sources for the native io
#
PDAL_ADD_TEST(pdal_io_bpf_test FILES io/bpf/BPFTest.cpp)
PDAL_ADD_TEST(pdal_io_buffer_test FILES io/buffer/BufferTest.cpp)
PDAL_ADD_TEST(pdal_io_faux_test FILES io/faux/FauxReaderTest.cpp)
PDAL_ADD_TEST(pdal_io_las_reader_test FILES io/las/LasReaderTest.cpp)
PDAL_ADD_TEST(pdal_io_las_writer_test FILES io/las/LasWriterTest.cpp)
PDAL_ADD_TEST(pdal_io_optech_test FILES io/optech/OptechReaderTest.cpp)
PDAL_ADD_TEST(pdal_io_ply_reader_test FILES io/ply/PlyReaderTest.cpp)
PDAL_ADD_TEST(pdal_io_ply_writer_test FILES io/ply/PlyWriterTest.cpp)
PDAL_ADD_TEST(pdal_io_qfit_test FILES io/qfit/QFITReaderTest.cpp)
PDAL_ADD_TEST(pdal_io_sbet_reader_test FILES io/sbet/SbetReaderTest.cpp)
PDAL_ADD_TEST(pdal_io_sbet_writer_test FILES io/sbet/SbetWriterTest.cpp)
PDAL_ADD_TEST(pdal_io_terrasolid_test FILES io/terrasolid/TerrasolidReaderTest.cpp)

#
# sources for the native filters
#
PDAL_ADD_TEST(pdal_filters_chipper_test FILES filters/ChipperTest.cpp)
PDAL_ADD_TEST(pdal_filters_colorization_test FILES filters/ColorizationFilterTest.cpp)
PDAL_ADD_TEST(pdal_filters_crop_test FILES filters/CropFilterTest.cpp)
PDAL_ADD_TEST(pdal_filters_decimation_test FILES filters/DecimationFilterTest.cpp)
PDAL_ADD_TEST(pdal_filters_ferry_test FILES filters/FerryFilterTest.cpp)
PDAL_ADD_TEST(pdal_filters_merge_test FILES filters/MergeTest.cpp)
PDAL_ADD_TEST(pdal_filters_reprojection_test FILES filters/ReprojectionFilterTest.cpp)
PDAL_ADD_TEST(pdal_filters_range_test FILES filters/RangeFilterTest.cpp)
PDAL_ADD_TEST(pdal_filters_sort_test FILES filters/SortFilterTest.cpp)
PDAL_ADD_TEST(pdal_filters_splitter_test FILES filters/SplitterTest.cpp)
PDAL_ADD_TEST(pdal_filters_stats_test FILES filters/StatsFilterTest.cpp)
PDAL_ADD_TEST(pdal_filters_transformation_test FILES filters/TransformationFilterTest.cpp)

#
# conditionally append apps/libxml2 sources
#
if (WITH_APPS)
    if (LASZIP_FOUND)
        PDAL_ADD_TEST(pdal_merge_test FILES apps/MergeTest.cpp)
    endif()
    PDAL_ADD_TEST(pc2pc_test FILES apps/pc2pcTest.cpp)

    if (BUILD_PIPELINE_TESTS)
        PDAL_ADD_TEST(pcpipeline_test FILES apps/pcpipelineTest.cpp)
    endif()
endif(WITH_APPS)

if(LIBXML2_FOUND)
    PDAL_ADD_TEST(xml_schema_test FILES XMLSchemaTest.cpp)
endif()
