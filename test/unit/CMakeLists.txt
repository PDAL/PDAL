###############################################################################
#
# test/unit/CMakeLists.txt controls building of libLAS unit tests suite
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
###############################################################################
SET(LIBLAS_UNIT_TEST liblas_test)

SET(LIBLAS_UNIT_TEST_SRC
    bounds_test.cpp
    common.cpp
    error_test.cpp
    guid_test.cpp
    header_test.cpp
    point_test.cpp
    reader_iterator_test.cpp
    reader_test.cpp
    spatialreference_test.cpp
    transform_test.cpp
    variablerecord_test.cpp
    writer_test.cpp
    zipreader_test.cpp
    zipwriter_test.cpp
    liblas_test_suite.cpp)

INCLUDE_DIRECTORIES(
    .
    ../../include
    ${GDAL_INCLUDE_DIR}
    ${GEOTIFF_INCLUDE_DIR})

ADD_EXECUTABLE(${LIBLAS_UNIT_TEST} ${LIBLAS_UNIT_TEST_SRC} )

set_target_properties(${LIBLAS_UNIT_TEST} PROPERTIES COMPILE_DEFINITIONS LAS_DLL_IMPORT)

TARGET_LINK_LIBRARIES(${LIBLAS_UNIT_TEST} 
    ${LIBLAS_LIB_NAME}
    ${ZLIB_LIBRARIES}
    ${TIFF_LIBRARY}
    ${GEOTIFF_LIBRARY}
    ${GDAL_LIBRARY}
    ${SPATIALINDEX_LIBRARY})

ADD_TEST(liblas_test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/liblas_test ${CMAKE_SOURCE_DIR}/test/data)
