#
# Greyhound plugin CMake configuration
#

if (WIN32)
add_definitions("-DARBITER_DLL_IMPORT")
endif()

#
# Need Winsock for htonl
#

PDAL_ADD_PLUGIN(reader_libname reader greyhound
    FILES
        io/CompressionStream.cpp
        io/GreyhoundReader.cpp
        io/GreyhoundCommon.cpp
        io/bounds.cpp
    LINK_WITH
        ${WINSOCK_LIBRARY}
    INCLUDES
        ${NLOHMANN_INCLUDE_DIR}
        ${PDAL_VENDOR_DIR}
)

PDAL_ADD_PLUGIN(writer_libname writer greyhound
    FILES
        io/CompressionStream.cpp
        io/GreyhoundWriter.cpp
        io/GreyhoundCommon.cpp
        io/bounds.cpp
    LINK_WITH
        ${WINSOCK_LIBRARY}
    INCLUDES
        ${NLOHMANN_INCLUDE_DIR}
        ${PDAL_VENDOR_DIR}
)


if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_io_greyhound_reader_test
        FILES
            test/GreyhoundReaderTest.cpp
        LINK_WITH
            ${reader_libname}
            ${writer_libname}
        INCLUDES
            ${NLOHMANN_INCLUDE_DIR}
            ${PDAL_VENDOR_DIR}
    )
endif()
