###############################################################################
#
# test/unit/CMakeLists.txt controls building of PDAL unit tests suite
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
###############################################################################
SET(PDAL_UNIT_TEST pdal_test)

configure_file(
    TestConfig.hpp.in
    "${CMAKE_CURRENT_BINARY_DIR}/TestConfig.hpp")


if (LIBXML2_FOUND)
SET (PDAL_UNITTEST_XMLSCHEMA_TEST XMLSchemaTest.cpp)
endif()

SET(PDAL_UNITTEST_TEST_SRC
    BoundsTest.cpp
    drivers/bpf/BPFTest.cpp
    drivers/buffer/BufferTest.cpp
#    drivers/caris/CarisReaderTest.cpp
    filters/ChipperTest.cpp
    ConfigTest.cpp
    filters/CropFilterTest.cpp
    filters/DecimationFilterTest.cpp
    filters/FerryFilterTest.cpp
    EnvironmentTest.cpp
    drivers/faux/FauxReaderTest.cpp
    FileUtilsTest.cpp
    filters/HexbinFilterTest.cpp
    filters/MergeTest.cpp
    drivers/las/LasReaderTest.cpp
    drivers/las/LasWriterTest.cpp
    LogTest.cpp
    MetadataTest.cpp
    OptionsTest.cpp
    PipelineManagerTest.cpp
    PointContextTest.cpp
    PointBufferTest.cpp
    drivers/qfit/QFITReaderTest.cpp
    filters/ReprojectionFilterTest.cpp
    filters/SortFilterTest.cpp
    drivers/sbet/SbetReaderTest.cpp
    drivers/sbet/SbetWriterTest.cpp
    SpatialReferenceTest.cpp
    filters/SplitterTest.cpp
#    StageFactoryTest.cpp
    filters/StatsFilterTest.cpp
    StreamFactoryTest.cpp
    SupportTest.cpp
#    drivers/terrasolid/TerraSolidReaderTest.cpp
#    drivers/text/TextWriterTest.cpp
    UserCallbackTest.cpp
    UtilsTest.cpp
    ${PDAL_UNITTEST_XMLSCHEMA_TEST}
    )

if(WITH_APPS)
  if(LIBXML2_FOUND)
    list(APPEND PDAL_UNITTEST_TEST_SRC
#      apps/pcinfoTest.cpp
    )
  endif(LIBXML2_FOUND)

  list(APPEND PDAL_UNITTEST_TEST_SRC
    apps/pc2pcTest.cpp
#    apps/pcpipelineTest.cpp
  )
endif(WITH_APPS)

set(PDAL_PLANG_TEST_SRC
    plang/PLangTest.cpp
    plang/PredicateFilterTest.cpp
    plang/ProgrammableFilterTest.cpp
)

SET(PDAL_UNITTEST_TEST_INC
    )

SET(PDAL_UNITTEST_CONFIG_SRC
    Support.cpp
    TestConfig.cpp
    main.cpp
    )

SET(PDAL_UNITTEST_CONFIG_INC
#    Support.hpp
#    TestConfig.hpp
    )


set(PDAL_GDAL_TEST_CPP
    GDALUtilsTest.cpp
    filters/ColorizationFilterTest.cpp
)

FOREACH(file ${PDAL_GDAL_TEST_CPP})
        SET(PDAL_UNITTEST_TEST_SRC "${PDAL_UNITTEST_TEST_SRC};${file}"
            CACHE INTERNAL "source files for test")
ENDFOREACH(file)

set(PDAL_UNITTEST_SOURCES "")
FOREACH(file ${PDAL_UNITTEST_TEST_SRC})
    SET(PDAL_UNITTEST_SOURCES "${PDAL_UNITTEST_SOURCES};${file}"
        CACHE INTERNAL "source files for test")
ENDFOREACH(file)

FOREACH(file ${PDAL_PLANG_TEST_SRC})
    SET(PDAL_UNITTEST_SOURCES "${PDAL_UNITTEST_SOURCES};${file}"
        CACHE INTERNAL "source files for test")
ENDFOREACH(file)

FOREACH(file ${PDAL_UNITTEST_CONFIG_SRC})
    SET(PDAL_UNITTEST_SOURCES "${PDAL_UNITTEST_SOURCES};${file}"
        CACHE INTERNAL "source files for test")
ENDFOREACH(file)

FOREACH(file ${PDAL_UNITTEST_CONFIG_INC})
    SET(PDAL_UNITTEST_SOURCES "${PDAL_UNITTEST_SOURCES};${file}"
        CACHE INTERNAL "source files for test")
ENDFOREACH(file)

source_group("Header Files" FILES ${PDAL_UNITTEST_TEST_INC})
source_group("Header Files\\config" FILES ${PDAL_UNITTEST_CONFIG_INC})
source_group("Source Files" FILES ${PDAL_UNITTEST_TEST_SRC})
source_group("Source Files\\config" FILES ${PDAL_UNITTEST_CONFIG_SRC})

INCLUDE_DIRECTORIES(
    .
    ${CMAKE_CURRENT_BINARY_DIR}
    ../../include
    ${GDAL_INCLUDE_DIR}
    ${GEOTIFF_INCLUDE_DIR}
    )
    #${ORACLE_INCLUDE_DIR})

ADD_EXECUTABLE(${PDAL_UNIT_TEST} ${PDAL_UNITTEST_SOURCES})

set_target_properties(${PDAL_UNIT_TEST} PROPERTIES COMPILE_DEFINITIONS
    PDAL_DLL_IMPORT)

if(WIN32)
    add_definitions("-DPDAL_DLL_EXPORT=1")
    if (MSVC)
        #add_definitions("-DBOOST_TEST_DYN_LINK")
    endif()
else()
    add_definitions("-DBOOST_TEST_DYN_LINK")
endif()

TARGET_LINK_LIBRARIES(  ${PDAL_UNIT_TEST} ${PDAL_LINKAGE}
                        ${PDAL_LIB_NAME} )

TARGET_LINK_LIBRARIES(  ${PDAL_UNIT_TEST} ${BOOST_LINKAGE}
                        ${Boost_LIBRARIES} )

ADD_TEST(pdal_test "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/pdal_test" "${PROJECT_SOURCE_DIR}/test/data" --catch_system_errors=no)
