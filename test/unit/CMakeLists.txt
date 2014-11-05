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
)

#
# sources for the native drivers
#
set(PDAL_UNITTEST_DRIVERS_SRCS
    drivers/bpf/BPFTest.cpp
    drivers/buffer/BufferTest.cpp
#    drivers/caris/CarisReaderTest.cpp
    drivers/faux/FauxReaderTest.cpp
    drivers/las/LasReaderTest.cpp
    drivers/las/LasWriterTest.cpp
    drivers/qfit/QFITReaderTest.cpp
    drivers/sbet/SbetReaderTest.cpp
    drivers/sbet/SbetWriterTest.cpp
#    drivers/terrasolid/TerraSolidReaderTest.cpp
#    drivers/text/TextWriterTest.cpp
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

PDAL_ADD_TEST(pdal_test "${PDAL_UNITTEST_SRCS}")
