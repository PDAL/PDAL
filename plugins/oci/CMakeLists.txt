#
# Oracle OCI plugin CMake configuration
#

find_package(Oracle REQUIRED)
include_directories(${ORACLE_INCLUDE_DIR})
set(OCI_CONNECTION "scott/tiger@localhost/test" CACHE STRING
    "OCI connection string <username/password@instance>")

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
