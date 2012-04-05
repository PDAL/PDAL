###############################################################################
#
# test/unit/CMakeLists.txt controls building of PDAL unit tests suite
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
###############################################################################
SET(PDAL_UNIT_TEST pdal_test)

if (LIBXML2_FOUND)
SET (PDAL_UNITTEST_XMLSCHEMA_TEST XMLSchemaTest.cpp)
endif()

SET(PDAL_UNITTEST_TEST_SRC
    apps/pc2pcTest.cpp
    apps/pcinfoTest.cpp
    apps/pcpipelineTest.cpp
    BoundsTest.cpp
    drivers/bpf/BPFTest.cpp
    filters/ByteSwapFilterTest.cpp
    filters/CacheFilterTest.cpp
    filters/ChipperTest.cpp
    filters/ColorFilterTest.cpp
    filters/ColorizationFilterTest.cpp
    ConfigTest.cpp
    filters/CropFilterTest.cpp
    filters/DecimationFilterTest.cpp
    DimensionLayoutTest.cpp
    DimensionTest.cpp
    EnvironmentTest.cpp
    drivers/faux/FauxReaderTest.cpp
    drivers/faux/FauxWriterTest.cpp
    FileUtilsTest.cpp
    filters/InPlaceReprojectionFilterTest.cpp
    drivers/las/LasReaderTest.cpp
    drivers/las/LasWriterTest.cpp
    MetadataTest.cpp
    filters/MosaicFilterTest.cpp
    OptionsTest.cpp
    PipelineManagerTest.cpp
    drivers/pipeline/PipelineReaderTest.cpp
    drivers/pipeline/PipelineWriterTest.cpp
    PointBufferCacheTest.cpp
    PointBufferTest.cpp
    drivers/qfit/QFITReaderTest.cpp
    RangeTest.cpp
    filters/ReprojectionFilterTest.cpp
    filters/ScalingFilterTest.cpp
    filters/SelectorFilterTest.cpp
    SchemaLayoutTest.cpp
    SchemaTest.cpp
    SpatialReferenceTest.cpp
    StageFactoryTest.cpp
    filters/StatsFilterTest.cpp
    StreamFactoryTest.cpp
    SupportTest.cpp
    drivers/terrasolid/TerraSolidTest.cpp
    drivers/text/TextWriterTest.cpp
    UserCallbackTest.cpp
    UtilsTest.cpp
    VectorTest.cpp
    ${PDAL_UNITTEST_XMLSCHEMA_TEST}
    )

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
    Support.hpp
    TestConfig.hpp
    )


if (WITH_ORACLE)
    set(PDAL_OCI_TEST_CPP drivers/oci/OCITest.cpp)

    FOREACH(file ${PDAL_OCI_TEST_CPP})
            SET(PDAL_UNITTEST_TEST_SRC "${PDAL_UNITTEST_TEST_SRC};${file}" CACHE INTERNAL "source files for test")
    ENDFOREACH(file)
    
endif (WITH_ORACLE)

if (WITH_GDAL)
    set(PDAL_GDAL_TEST_CPP
        GDALUtilsTest.cpp
        drivers/nitf/NitfReaderTest.cpp
        drivers/nitf/NitfWriterTest.cpp
        )

    FOREACH(file ${PDAL_GDAL_TEST_CPP})
            SET(PDAL_UNITTEST_TEST_SRC "${PDAL_UNITTEST_TEST_SRC};${file}" CACHE INTERNAL "source files for test")
    ENDFOREACH(file)
    
endif (WITH_GDAL)

set(PDAL_UNITTEST_SOURCES "")
FOREACH(file ${PDAL_UNITTEST_TEST_SRC})
        SET(PDAL_UNITTEST_SOURCES "${PDAL_UNITTEST_SOURCES};${file}" CACHE INTERNAL "source files for test")
ENDFOREACH(file)

FOREACH(file ${PDAL_UNITTEST_TEST_SRC})
        SET(PDAL_UNITTEST_SOURCES "${PDAL_UNITTEST_SOURCES};${file}" CACHE INTERNAL "source files for test")
ENDFOREACH(file)

FOREACH(file ${PDAL_PLANG_TEST_SRC})
        SET(PDAL_UNITTEST_SOURCES "${PDAL_UNITTEST_SOURCES};${file}" CACHE INTERNAL "source files for test")
ENDFOREACH(file)

FOREACH(file ${PDAL_UNITTEST_CONFIG_SRC})
        SET(PDAL_UNITTEST_SOURCES "${PDAL_UNITTEST_SOURCES};${file}" CACHE INTERNAL "source files for test")
ENDFOREACH(file)
FOREACH(file ${PDAL_UNITTEST_CONFIG_INC})
        SET(PDAL_UNITTEST_SOURCES "${PDAL_UNITTEST_SOURCES};${file}" CACHE INTERNAL "source files for test")
ENDFOREACH(file)

source_group("Header Files" FILES ${PDAL_UNITTEST_TEST_INC})
source_group("Header Files\\config" FILES ${PDAL_UNITTEST_CONFIG_INC})
source_group("Source Files" FILES ${PDAL_UNITTEST_TEST_SRC})
source_group("Source Files\\config" FILES ${PDAL_UNITTEST_CONFIG_SRC})


INCLUDE_DIRECTORIES(
    .
    ../../include
    ${GDAL_INCLUDE_DIR}
    ${GEOTIFF_INCLUDE_DIR})

ADD_EXECUTABLE(${PDAL_UNIT_TEST} ${PDAL_UNITTEST_SOURCES})

set_target_properties(${PDAL_UNIT_TEST} PROPERTIES COMPILE_DEFINITIONS PDAL_DLL_IMPORT)

if(WIN32)
    add_definitions("-DPDAL_DLL_EXPORT=1")
  if (MSVC)
    #add_definitions("-DBOOST_TEST_DYN_LINK")
  endif()
else()
    # Our own static build of embedded boost should do BOOST_TEST_DYN_LINK 
    # when it is compiled, but regular static builds of Boost will 
    # not do this. 
    if(Boost_PROGRAM_OPTIONS_LIBRARY MATCHES "\\.a$" )
        # pass, we can compile static
        message(STATUS "Static boost for boost::unit_test, not setting BOOST_TEST_DYN_LINK")
    else()
        message(STATUS "No non-embed static boost library found, setting BOOST_TEST_DYN_LINK")
        add_definitions("-DBOOST_TEST_DYN_LINK")
    endif()
endif()


	TARGET_LINK_LIBRARIES(${PDAL_UNIT_TEST} 
		${PDAL_LIB_NAME})

ADD_TEST(pdal_test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/pdal_test ${PROJECT_SOURCE_DIR}/test/data)
