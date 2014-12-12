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

set(deps ${ORACLE_LIBRARY})
PDAL_ADD_PLUGIN(oci_reader_lib_name reader oci "${srcs}" "${incs}" "${deps}")

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

set(deps ${ORACLE_LIBRARY})
PDAL_ADD_PLUGIN(oci_writer_lib_name writer oci "${srcs}" "${incs}" "${deps}")

#
# OCI tests
#
if(BUILD_OCI_TESTS)
    set(srcs
        test/OCITest.cpp
    )

    set(deps ${oci_reader_lib_name} ${oci_writer_lib_name})
    PDAL_ADD_TEST(ocitest "${srcs}" "${deps}")
endif()
