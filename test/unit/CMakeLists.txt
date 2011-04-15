###############################################################################
#
# test/unit/CMakeLists.txt controls building of libPC unit tests suite
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
###############################################################################
SET(LIBPC_UNIT_TEST libpc_test)

SET(LIBPC_UNITTEST_TEST_SRC
    BoundsTest.cpp
    CacheFilterTest.cpp
    ChipperTest.cpp
    ColorTest.cpp
    ColorFilterTest.cpp
    ConfigTest.cpp
    CropFilterTest.cpp
    DecimationFilterTest.cpp
    DimensionLayoutTest.cpp
    DimensionTest.cpp
    FauxReaderTest.cpp
    FauxWriterTest.cpp
    LasReaderTest.cpp
    LasWriterTest.cpp
    LiblasReaderTest.cpp
    LiblasWriterTest.cpp
    MosaicFilterTest.cpp
    PointBufferCacheTest.cpp
    PointBufferTest.cpp
    RangeTest.cpp
    SchemaLayoutTest.cpp
    SchemaTest.cpp
    SignallerTest.cpp
    UtilsTest.cpp
    VectorTest.cpp
    )

SET(LIBPC_UNITTEST_TEST_INC
    )
    
SET(LIBPC_UNITTEST_CONFIG_SRC
    Support.cpp
    TestConfig.cpp
    main.cpp
    )

SET(LIBPC_UNITTEST_CONFIG_INC
    Support.hpp
    TestConfig.hpp
    )


if (WITH_ORACLE)
    set(LIBPC_OCI_TEST_CPP OCITest.cpp)

    FOREACH(file ${LIBPC_OCI_TEST_CPP})
            SET(LIBPC_UNITTEST_TEST_SRC "${LIBPC_UNITTEST_TEST_SRC};${file}" CACHE INTERNAL "source files for test")
    ENDFOREACH(file)
    
endif (WITH_ORACLE)


set(LIBPC_UNITTEST_SOURCES "")
FOREACH(file ${LIBPC_UNITTEST_TEST_SRC})
        SET(LIBPC_UNITTEST_SOURCES "${LIBPC_UNITTEST_SOURCES};${file}" CACHE INTERNAL "source files for test")
ENDFOREACH(file)
FOREACH(file ${LIBPC_UNITTEST_TEST_INC})
        SET(LIBPC_UNITTEST_SOURCES "${LIBPC_UNITTEST_SOURCES};${file}" CACHE INTERNAL "source files for test")
ENDFOREACH(file)
FOREACH(file ${LIBPC_UNITTEST_CONFIG_SRC})
        SET(LIBPC_UNITTEST_SOURCES "${LIBPC_UNITTEST_SOURCES};${file}" CACHE INTERNAL "source files for test")
ENDFOREACH(file)
FOREACH(file ${LIBPC_UNITTEST_CONFIG_INC})
        SET(LIBPC_UNITTEST_SOURCES "${LIBPC_UNITTEST_SOURCES};${file}" CACHE INTERNAL "source files for test")
ENDFOREACH(file)

source_group("Header Files" FILES ${LIBPC_UNITTEST_TEST_INC})
source_group("Header Files\\config" FILES ${LIBPC_UNITTEST_CONFIG_INC})
source_group("Source Files" FILES ${LIBPC_UNITTEST_TEST_SRC})
source_group("Source Files\\config" FILES ${LIBPC_UNITTEST_CONFIG_SRC})


INCLUDE_DIRECTORIES(
    .
    ../../include
    ${GDAL_INCLUDE_DIR}
    ${GEOTIFF_INCLUDE_DIR})

ADD_EXECUTABLE(${LIBPC_UNIT_TEST} ${LIBPC_UNITTEST_SOURCES})

set_target_properties(${LIBPC_UNIT_TEST} PROPERTIES COMPILE_DEFINITIONS LIBPC_DLL_IMPORT)

if(WIN32)
    add_definitions("-DLIBPC_DLL_EXPORT=1")
  if (MSVC)
    #add_definitions("-DBOOST_TEST_DYN_LINK")
  endif()
else()
    if(Boost_PROGRAM_OPTIONS_LIBRARY MATCHES "\\.a$")
        # pass, we can compile static
    else()
        add_definitions("-DBOOST_TEST_DYN_LINK")
    endif()
endif()

TARGET_LINK_LIBRARIES(${LIBPC_UNIT_TEST} 
    ${LIBPC_LIB_NAME}
    ${LIBLAS_LIBRARY}
    ${ZLIB_LIBRARIES}
    ${TIFF_LIBRARY}
    ${GEOTIFF_LIBRARY}
    ${GDAL_LIBRARY}
    ${SPATIALINDEX_LIBRARY})

ADD_TEST(libpc_test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/libpc_test ${PROJECT_SOURCE_DIR}/test/data)
