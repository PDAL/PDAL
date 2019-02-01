#
# Oracle OCI plugin CMake configuration
#
find_package(Oracle REQUIRED)

set(OCI_CONNECTION "scott/tiger@localhost/test" CACHE STRING
    "OCI connection string <username/password@instance>")
#
# OCI Reader
#
PDAL_ADD_PLUGIN(reader_libname reader oci
    FILES
        io/OciCommon.cpp
        io/OciReader.cpp
        io/OciWrapper.cpp
    LINK_WITH
        ${ORACLE_LIBRARY}
        ${GDAL_LIBRARY}
        ${LIBXML2_LIBRARIES}
    INCLUDES
        ${ORACLE_INCLUDE_DIR}
        ${PDAL_VENDOR_DIR}/pdalboost
        ${LIBXML2_INCLUDE_DIR}
)
#
# OCI Writer
#
PDAL_ADD_PLUGIN(writer_libname writer oci
    FILES
        io/OciCommon.cpp
        io/OciWrapper.cpp
        io/OciWriter.cpp
    LINK_WITH
        ${ORACLE_LIBRARY}
        ${GDAL_LIBRARY}
        ${LIBXML2_LIBRARIES}
    INCLUDES
        ${ORACLE_INCLUDE_DIR}
        ${PDAL_VENDOR_DIR}/pdalboost
        ${LIBXML2_INCLUDE_DIR}
)
#
# OCI tests
#
if(BUILD_OCI_TESTS)
    PDAL_ADD_TEST(ocitest
        FILES
            test/OCITest.cpp
        LINK_WITH
            ${reader_libname}
            ${writer_libname}
            ${GDAL_LIBRARY}
            ${LIBXML2_LIBRARY}
        INCLUDES
            ${ORACLE_INCLUDE_DIR}
            ${LIBXML2_INCLUDE_DIR}
    )
endif()
