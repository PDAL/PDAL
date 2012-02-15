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
    pc2pcTest.cpp
    pcinfoTest.cpp
    pcpipelineTest.cpp
    BoundsTest.cpp
    ByteSwapFilterTest.cpp
    CacheFilterTest.cpp
    ChipperTest.cpp
    ColorFilterTest.cpp
    ColorizationFilterTest.cpp
    ConfigTest.cpp
    CropFilterTest.cpp
    DecimationFilterTest.cpp
    DimensionLayoutTest.cpp
    DimensionTest.cpp
    FauxReaderTest.cpp
    FauxWriterTest.cpp
    FileUtilsTest.cpp
    InPlaceReprojectionFilterTest.cpp
    LasReaderTest.cpp
    LasWriterTest.cpp
    MosaicFilterTest.cpp
    OptionsTest.cpp
    PipelineManagerTest.cpp
    PipelineReaderTest.cpp
    PipelineWriterTest.cpp
    PointBufferCacheTest.cpp
    PointBufferTest.cpp
    QFITReaderTest.cpp
    RangeTest.cpp
    ReprojectionFilterTest.cpp
    ScalingFilterTest.cpp
    SelectorFilterTest.cpp
    SchemaLayoutTest.cpp
    SchemaTest.cpp
    SpatialReferenceTest.cpp
    StageFactoryTest.cpp
    StatsFilterTest.cpp
    StreamManagerTest.cpp
    SupportTest.cpp
    TerraSolidTest.cpp
    TextWriterTest.cpp
    UserCallbackTest.cpp
    UtilsTest.cpp
    VectorTest.cpp
    ${PDAL_UNITTEST_XMLSCHEMA_TEST}
    )

set(PDAL_PLANG_TEST_SRC
    PLangParserTest.cpp
    PLangInternalTest.cpp
    PredicateFilterTest.cpp
    ProgrammableFilterTest.cpp
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
    set(PDAL_OCI_TEST_CPP OCITest.cpp)

    FOREACH(file ${PDAL_OCI_TEST_CPP})
            SET(PDAL_UNITTEST_TEST_SRC "${PDAL_UNITTEST_TEST_SRC};${file}" CACHE INTERNAL "source files for test")
    ENDFOREACH(file)
    
endif (WITH_ORACLE)


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

if(LIBLAS_FOUND)
	TARGET_LINK_LIBRARIES(${PDAL_UNIT_TEST} 
		${PDAL_LIB_NAME}
		${LIBLAS_LIBRARY}
		${ZLIB_LIBRARIES}
		${TIFF_LIBRARY}
		${GEOTIFF_LIBRARY}
		${GDAL_LIBRARY}
		${SPATIALINDEX_LIBRARY})
else()
	TARGET_LINK_LIBRARIES(${PDAL_UNIT_TEST} 
		${PDAL_LIB_NAME}
		${ZLIB_LIBRARIES}
		${TIFF_LIBRARY}
		${GEOTIFF_LIBRARY}
		${GDAL_LIBRARY}
		${SPATIALINDEX_LIBRARY})
endif()	

ADD_TEST(pdal_test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/pdal_test ${PROJECT_SOURCE_DIR}/test/data)
