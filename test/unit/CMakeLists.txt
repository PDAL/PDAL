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
    BoundsTest.cpp
    drivers/bpf/BPFTest.cpp
#    drivers/caris/CarisReaderTest.cpp
    filters/ChipperTest.cpp
    ConfigTest.cpp
    filters/CropFilterTest.cpp
    filters/DecimationFilterTest.cpp
    DimensionTest.cpp
    EnvironmentTest.cpp
    drivers/faux/FauxReaderTest.cpp
    FileUtilsTest.cpp
    filters/HexbinFilterTest.cpp
    filters/InPlaceReprojectionFilterTest.cpp
    drivers/las/LasReaderTest.cpp
    drivers/las/LasWriterTest.cpp
    LogTest.cpp
    MetadataTest.cpp
#    filters/MosaicFilterTest.cpp
    OptionsTest.cpp
    PipelineManagerTest.cpp
    PointBufferTest.cpp
#    drivers/qfit/QFITReaderTest.cpp
    RangeTest.cpp
    filters/ReprojectionFilterTest.cpp
    drivers/sbet/SbetReaderTest.cpp
    drivers/sbet/SbetWriterTest.cpp
    filters/ScalingFilterTest.cpp
    filters/SelectorFilterTest.cpp
#    SchemaLayoutTest.cpp
#    SchemaTest.cpp
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
    VectorTest.cpp
    ${PDAL_UNITTEST_XMLSCHEMA_TEST}
    )

if(WITH_APPS)
  if(LIBXML2_FOUND)
    list(APPEND PDAL_UNITTEST_TEST_SRC 
      apps/pcinfoTest.cpp
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


if (WITH_ORACLE)
if (NOT USE_PDAL_PLUGIN_OCI)
   set(PDAL_OCI_TEST_CPP drivers/oci/OCITest.cpp)

   FOREACH(file ${PDAL_OCI_TEST_CPP})
           SET(PDAL_UNITTEST_TEST_SRC "${PDAL_UNITTEST_TEST_SRC};${file}" CACHE INTERNAL "source files for test")
   ENDFOREACH(file)
endif()

endif (WITH_ORACLE)

if (WITH_SQLITE)
    if (NOT USE_PDAL_PLUGIN_SQLITE)
       set(PDAL_SQLITE_TEST_CPP drivers/sqlite/SqliteTest.cpp)

       FOREACH(file ${PDAL_SQLITE_TEST_CPP})
           SET(PDAL_UNITTEST_TEST_SRC "${PDAL_UNITTEST_TEST_SRC};${file}"
               CACHE INTERNAL "source files for test")
       ENDFOREACH(file)
    endif()
endif (WITH_SQLITE)

if (WITH_GDAL)
    set(PDAL_GDAL_TEST_CPP
        GDALUtilsTest.cpp
        filters/ColorizationFilterTest.cpp
        drivers/nitf/NitfReaderTest.cpp
        )

    FOREACH(file ${PDAL_GDAL_TEST_CPP})
            SET(PDAL_UNITTEST_TEST_SRC "${PDAL_UNITTEST_TEST_SRC};${file}"
                CACHE INTERNAL "source files for test")
    ENDFOREACH(file)
endif (WITH_GDAL)

if (WITH_NITRO)
    set(PDAL_NITRO_TEST_CPP
        drivers/nitf/NitfWriterTest.cpp
        )

    FOREACH(file ${PDAL_NITRO_TEST_CPP})
        SET(PDAL_UNITTEST_TEST_SRC "${PDAL_UNITTEST_TEST_SRC};${file}"
            CACHE INTERNAL "source files for test")
    ENDFOREACH(file)
endif (WITH_NITRO)

if (WITH_PCL AND WITH_PCL_TEST)
    set(PDAL_PCL_TEST_CPP
        filters/PCLBlockFilterTest.cpp
        )

    FOREACH(file ${PDAL_PCL_TEST_CPP})
        SET(PDAL_UNITTEST_TEST_SRC "${PDAL_UNITTEST_TEST_SRC};${file}"
            CACHE INTERNAL "source files for test")
    ENDFOREACH(file)
endif (WITH_PCL AND WITH_PCL_TEST)

if (WITH_HDF5)
    set(PDAL_ICEBRIDGE_TEST_CPP
        drivers/icebridge/IcebridgeReaderTest.cpp
        )

    FOREACH(file ${PDAL_ICEBRIDGE_TEST_CPP})
        SET(PDAL_UNITTEST_TEST_SRC "${PDAL_UNITTEST_TEST_SRC};${file}"
            CACHE INTERNAL "source files for test")
    ENDFOREACH(file)
endif(WITH_HDF5)

if (WITH_PGPOINTCLOUD AND WITH_PGPOINTCLOUD_TESTS)
    set(PDAL_PGPOINTCLOUD_TEST_CPP
        drivers/pgpointcloud/PgpointcloudWriterTest.cpp
        )

    set(PGPOINTCLOUD_TEST_DB_HOST localhost CACHE STRING "Postgres test database host")
    set(PGPOINTCLOUD_TEST_DB_PORT 5432 CACHE STRING "Postgres test database port")
    set(PGPOINTCLOUD_TEST_DB_NAME pdal_test CACHE STRING
        "Postgres test database name, must exist and must be able to create databases")
    set(PGPOINTCLOUD_TEST_DB_TEMPNAME pdal_test_tmp CACHE STRING "Postgres test database temp database name")

    configure_file(
        drivers/pgpointcloud/Pgtest-Support.hpp.in
        ${CMAKE_CURRENT_BINARY_DIR}/drivers/pgpointcloud/Pgtest-Support.hpp
        )

    set(PDAL_UNITTEST_TEST_SRC
        ${PDAL_UNITTEST_TEST_SRC}
        ${PDAL_PGPOINTCLOUD_TEST_CPP}
        CACHE INTERNAL "source files for test"
        )
endif (WITH_PGPOINTCLOUD AND WITH_PGPOINTCLOUD_TESTS)

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
    ${ORACLE_INCLUDE_DIR})

ADD_EXECUTABLE(${PDAL_UNIT_TEST} ${PDAL_UNITTEST_SOURCES})

set_target_properties(${PDAL_UNIT_TEST} PROPERTIES COMPILE_DEFINITIONS
    PDAL_DLL_IMPORT)

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

TARGET_LINK_LIBRARIES(  ${PDAL_UNIT_TEST} ${PDAL_LINKAGE}
                        ${PDAL_LIB_NAME} )

TARGET_LINK_LIBRARIES(  ${PDAL_UNIT_TEST} ${BOOST_LINKAGE}
                        ${Boost_LIBRARIES} )

ADD_TEST(pdal_test "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/pdal_test" "${PROJECT_SOURCE_DIR}/test/data" --catch_system_errors=no)
