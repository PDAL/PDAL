###############################################################################
#
# test/unit/CMakeLists.txt controls building of libPC unit tests suite
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
###############################################################################
SET(LIBPC_UNIT_TEST libpc_test)

SET(LIBPC_UNIT_TEST_SRC
	BoundsTest.cpp
	ColorTest.cpp
	DimensionTest.cpp
	RangeTest.cpp
	UtilsTest.cpp
	VectorTest.cpp
    main.cpp)

INCLUDE_DIRECTORIES(
    .
    ../../include
    ${GDAL_INCLUDE_DIR}
    ${GEOTIFF_INCLUDE_DIR})

ADD_EXECUTABLE(${LIBPC_UNIT_TEST} ${LIBPC_UNIT_TEST_SRC} )

set_target_properties(${LIBPC_UNIT_TEST} PROPERTIES COMPILE_DEFINITIONS LIBPC_DLL_IMPORT)

if(WIN32)
    add_definitions("-DLIBPC_DLL_EXPORT=1")
endif()

TARGET_LINK_LIBRARIES(${LIBPC_UNIT_TEST} 
    ${LIBPC_LIB_NAME}
    ${ZLIB_LIBRARIES}
    ${TIFF_LIBRARY}
    ${GEOTIFF_LIBRARY}
    ${GDAL_LIBRARY}
    ${SPATIALINDEX_LIBRARY})

ADD_TEST(libpc_test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/libpc_test ${CMAKE_SOURCE_DIR}/test/data)
