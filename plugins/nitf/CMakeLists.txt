#
# NITF plugin CMake configuration
#
include(${PDAL_CMAKE_DIR}/nitro.cmake)
if (NOT NITRO_FOUND)
    message(FATAL_ERROR "Can't find NITRO support required by NITF.")
endif()

if (WIN32)
    add_definitions("-DNOMINMAX")
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
        ${NITRO_LIBRARIES}
    INCLUDES
        ${PDAL_VENDOR_DIR}/pdalboost
        ${PDAL_JSONCPP_INCLUDE_DIR}
        ${ROOT_DIR}
)
if (WIN32)
target_compile_definitions(${reader_libname} PRIVATE
    NOMINMAX)
endif()
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
        ${NITRO_LIBRARIES}
        ${GDAL_LIBRARY}
    INCLUDES
        ${PDAL_VENDOR_DIR}/pdalboost
        ${PDAL_JSONCPP_INCLUDE_DIR}
        ${ROOT_DIR}
)
if (WIN32)
target_compile_definitions(${writer_libname} PRIVATE
    NOMINMAX)
endif()

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_io_nitf_writer_test
        FILES
            test/NitfWriterTest.cpp
        LINK_WITH
            ${writer_libname}
        INCLUDES
            ${ROOT_DIR}
            ${PDAL_JSONCPP_INCLUDE_DIR}
    )

    PDAL_ADD_TEST(pdal_io_nitf_reader_test
        FILES
            test/NitfReaderTest.cpp
        LINK_WITH
            ${reader_libname}
        INCLUDES
            ${ROOT_DIR}
            ${PDAL_JSONCPP_INCLUDE_DIR}
    )
endif()
