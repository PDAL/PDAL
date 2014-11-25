###############################################################################
#
# test/unit/CMakeLists.txt controls building of PDAL unit tests suite
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
###############################################################################

configure_file(
    TestConfig.hpp.in
    "${CMAKE_CURRENT_BINARY_DIR}/TestConfig.hpp"
)

if (PDAL_HAVE_LAZPERF)
    SET(PDAL_LAZPERF_SRC CompressionTest.cpp)
endif()

SET(PDAL_UNITTEST_TEST_SRC "")
#
# sources for the base library
#
set(PDAL_UNITTEST_BASE_SRCS
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
#    StageFactoryTest.cpp
    StreamFactoryTest.cpp
    SupportTest.cpp
    UserCallbackTest.cpp
    UtilsTest.cpp
    ${PDAL_UNITTEST_XMLSCHEMA_TEST}
    ${PDAL_LAZPERF_SRC}
)

#
# sources for the native io
#
set(PDAL_UNITTEST_DRIVERS_SRCS
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

#
# sources for the native filters
#
set(PDAL_UNITTEST_FILTERS_SRCS
    filters/ChipperTest.cpp
    filters/ColorizationFilterTest.cpp
    filters/CropFilterTest.cpp
    filters/DecimationFilterTest.cpp
    filters/FerryFilterTest.cpp
    filters/MergeTest.cpp
    filters/ReprojectionFilterTest.cpp
    filters/SortFilterTest.cpp
    filters/SplitterTest.cpp
    filters/StatsFilterTest.cpp
)

#
# sources for plang
#
set(PDAL_PLANG_TEST_SRC
    plang/PLangTest.cpp
    plang/PredicateFilterTest.cpp
    plang/ProgrammableFilterTest.cpp
)

#
# the combined test sources
#
set(PDAL_UNITTEST_SRCS
    ${PDAL_UNITTEST_BASE_SRCS}
    ${PDAL_UNITTEST_DRIVERS_SRCS}
    ${PDAL_UNITTEST_FILTERS_SRCS}
    ${PDAL_PLANG_TEST_SRC}
    ${PDAL_TARGET_OBJECTS}
)

#
# conditionally append apps/libxml2 sources
#
if(WITH_APPS)
    if(LIBXML2_FOUND)
        list(APPEND PDAL_UNITTEST_SRCS
#            apps/pcinfoTest.cpp
        )
    endif(LIBXML2_FOUND)

    list(APPEND PDAL_UNITTEST_SRCS
        apps/pc2pcTest.cpp
#        apps/pcpipelineTest.cpp
    )
endif(WITH_APPS)

if(LIBXML2_FOUND)
    list(APPEND PDAL_UNITTEST_SRCS XMLSchemaTest.cpp)
endif()

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
    ${PROJECT_SOURCE_DIR}/filters/decimation
    ${PROJECT_SOURCE_DIR}/filters/ferry
    ${PROJECT_SOURCE_DIR}/filters/mortonorder
    ${PROJECT_SOURCE_DIR}/filters/splitter
)
set(deps ${PDAL_LIB_NAME})
PDAL_ADD_TEST(pdal_test "${PDAL_UNITTEST_SRCS}" "${deps}")
