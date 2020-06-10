#
# NITF plugin CMake configuration
#
include(${PDAL_CMAKE_DIR}/nitro.cmake)
if (NOT NITRO_FOUND)
    message(FATAL_ERROR "Can't find NITRO support required by NITF.")
endif()

PDAL_ADD_PLUGIN(reader_libname reader nitf
    FILES
        io/NitfFileReader.cpp
        io/MetadataReader.cpp
        io/tre_plugins.cpp
        io/NitfReader.cpp
    LINK_WITH
        ${NITRO_LIBRARIES}
)
if (WIN32)
    target_compile_definitions(${reader_libname}
        PRIVATE
            WIN32
    )
endif()

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
        ${GDAL_INCLUDE_DIR}
)
if (WIN32)
    target_compile_definitions(${writer_libname}
        PRIVATE
            WIN32
    )
endif()

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_io_nitf_writer_test
        FILES
            test/NitfWriterTest.cpp
        LINK_WITH
            ${writer_libname}
    )

    PDAL_ADD_TEST(pdal_io_nitf_reader_test
        FILES
            test/NitfReaderTest.cpp
        LINK_WITH
            ${reader_libname}
    )
endif()
