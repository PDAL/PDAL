#
# NITF plugin CMake configuration
#
include(${PDAL_CMAKE_DIR}/nitro.cmake)
if (NOT NITRO_FOUND)
    message(FATAL_ERROR "Can't find NITRO support required by NITF.")
endif()

#
# NITF Reader
#
PDAL_ADD_PLUGIN(reader_libname reader nitf
    FILES
        io/NitfFileReader.cpp
        io/MetadataReader.cpp
        io/tre_plugins.cpp
        io/NitfReader.cpp
    LINK_WITH
        ${NITRO_LIBRARIES})
target_include_directories(${reader_libname} PRIVATE
    ${PDAL_VENDOR_DIR}/pdalboost
    ${ROOT_DIR})

#
# NITF Writer
#

PDAL_ADD_PLUGIN(writer_libname writer nitf
    FILES
        io/NitfFileWriter.cpp
        io/NitfWriter.cpp
        io/MetadataReader.cpp
        io/tre_plugins.cpp
    LINK_WITH
        ${NITRO_LIBRARIES})
target_include_directories(${writer_libname} PRIVATE
    ${PDAL_VENDOR_DIR}/pdalboost
    ${ROOT_DIR})

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_io_nitf_writer_test
        FILES
            test/NitfWriterTest.cpp
        LINK_WITH ${writer_libname})
    target_include_directories(pdal_io_nitf_writer_test PRIVATE
        ${ROOT_DIR})

    PDAL_ADD_TEST(pdal_io_nitf_reader_test
        FILES
            test/NitfReaderTest.cpp
        LINK_WITH ${reader_libname})
    target_include_directories(pdal_io_nitf_reader_test PRIVATE
        ${ROOT_DIR})
endif()
