#
# Oracle OCI plugin CMake configuration
#

find_package(Oracle REQUIRED)
message(STATUS "Building with Oracle support")

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/oci)
include_directories(${ORACLE_INCLUDE_DIR})
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
)

#
# OCI Reader
#
set(srcs
    io/OciCommon.cpp
    io/OciReader.cpp
    io/OciWrapper.cpp
)

set(incs
    io/OciCommon.hpp
    io/OciReader.hpp
    io/OciWrapper.h
)

PDAL_ADD_PLUGIN(reader_libname reader oci
    FILES "${srcs}" "${incs}"
    LINK_WITH ${ORACLE_LIBRARY})

#
# OCI Writer
#
set(srcs
    io/OciCommon.cpp
    io/OciWrapper.cpp
    io/OciWriter.cpp
)

set(incs
    io/OciCommon.hpp
    io/OciWrapper.h
    io/OciWriter.hpp
)

PDAL_ADD_PLUGIN(writer_libname writer oci
    FILES "${srcs}" "${incs}"
    LINK_WITH ${ORACLE_LIBRARY})

#
# OCI tests
#
if(BUILD_OCI_TESTS)
    PDAL_ADD_TEST(ocitest
        FILES test/OCITest.cpp
        LINK_WITH ${reader_libname} ${writer_libname})
endif()
