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
    ${PROJECT_SOURCE_DIR}/filters/decimation
    ${PROJECT_SOURCE_DIR}/filters/ferry
    ${PROJECT_SOURCE_DIR}/filters/mortonorder
    ${PROJECT_SOURCE_DIR}/filters/reprojection
    ${PROJECT_SOURCE_DIR}/filters/sort
    ${PROJECT_SOURCE_DIR}/filters/splitter
    ${PROJECT_SOURCE_DIR}/filters/stats
    ${PROJECT_SOURCE_DIR}/kernels/info
)

set(srcs
    BoundsTest.cpp
    ConfigTest.cpp
    EnvironmentTest.cpp
    FileUtilsTest.cpp
    GDALUtilsTest.cpp
    LogTest.cpp
    MetadataTest.cpp
    OptionsTest.cpp
    PipelineManagerTest.cpp
    PointBufferTest.cpp
    PointContextTest.cpp
    SpatialReferenceTest.cpp
    #StageFactoryTest.cpp # this has been disabled for quite a while, is it OBE?
    StreamFactoryTest.cpp
    SupportTest.cpp
    UserCallbackTest.cpp
    UtilsTest.cpp
)
PDAL_ADD_TEST(pdal_base_test "${srcs}" ${PDAL_LIB_NAME})

if (PDAL_HAVE_LAZPERF)
    set(srcs CompressionTest.cpp)
    PDAL_ADD_TEST(lazperf_test "${srcs}" ${PDAL_LIB_NAME})
endif()

#
# sources for the native io
#
set(srcs
    io/bpf/BPFTest.cpp
    io/buffer/BufferTest.cpp
    io/faux/FauxReaderTest.cpp
    io/las/LasReaderTest.cpp
    io/las/LasWriterTest.cpp
    io/qfit/QFITReaderTest.cpp
    io/sbet/SbetReaderTest.cpp
    io/sbet/SbetWriterTest.cpp
#    io/terrasolid/TerraSolidReaderTest.cpp
#    io/text/TextWriterTest.cpp
)
PDAL_ADD_TEST(pdal_io_test "${srcs}" ${PDAL_LIB_NAME})

#
# sources for the native filters
#
set(srcs
    filters/ChipperTest.cpp
    filters/ColorizationFilterTest.cpp
    filters/DecimationFilterTest.cpp
    filters/FerryFilterTest.cpp
    filters/MergeTest.cpp
    filters/ReprojectionFilterTest.cpp
    filters/SortFilterTest.cpp
    filters/SplitterTest.cpp
    filters/StatsFilterTest.cpp
)
PDAL_ADD_TEST(pdal_filters_test "${srcs}" ${PDAL_LIB_NAME})

#
# sources for plang
#
if (PDAL_HAVE_PYTHON)
    set(srcs
        plang/PLangTest.cpp
        plang/PredicateFilterTest.cpp
        plang/ProgrammableFilterTest.cpp
    )
    PDAL_ADD_TEST(pdal_plang_test "${srcs}" ${PDAL_LIB_NAME})
endif()

#
# conditionally append apps/libxml2 sources
#
if(WITH_APPS)
#    if(LIBXML2_FOUND)
#        list(APPEND PDAL_UNITTEST_SRCS
#            apps/pcinfoTest.cpp
#        )
#    endif(LIBXML2_FOUND)

#    list(APPEND PDAL_UNITTEST_SRCS
#        apps/pc2pcTest.cpp
#        apps/pcpipelineTest.cpp
#    )
    set(srcs apps/pc2pcTest.cpp)
    PDAL_ADD_TEST(pc2pc_test "${srcs}" ${PDAL_LIB_NAME})
endif(WITH_APPS)

if(LIBXML2_FOUND)
    set(srcs XMLSchemaTest.cpp)
    PDAL_ADD_TEST(xml_schema_test "${srcs}" ${PDAL_LIB_NAME})
endif()
