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
set(srcs
    io/NitfFileReader.cpp
    io/MetadataReader.cpp
    io/tre_plugins.cpp
    io/NitfReader.cpp
)

set(incs
    io/NitfFileReader.hpp
    io/MetadataReader.hpp
    io/tre_plugins.hpp
    io/NitfReader.hpp
)

PDAL_ADD_PLUGIN(reader_libname reader nitf
    FILES "${srcs}" "${incs}"
    LINK_WITH ${NITRO_LIBRARIES})
target_include_directories(${reader_libname} PRIVATE
    ${ROOT_DIR}/vendor/pdalboost
    ${PDAL_IO_DIR})

#
# NITF Writer
#
set(srcs
    io/NitfFileWriter.cpp
    io/NitfWriter.cpp
    io/MetadataReader.cpp
    io/tre_plugins.cpp
)

set(incs
    io/NitfFileWriter.hpp
    io/NitfWriter.hpp
    io/MetadataReader.hpp
    io/tre_plugins.hpp
)

PDAL_ADD_PLUGIN(writer_libname writer nitf
    FILES "${srcs}" "${incs}"
    LINK_WITH ${NITRO_LIBRARIES})
target_include_directories(${writer_libname} PRIVATE
    ${ROOT_DIR}/vendor/pdalboost
    ${PDAL_IO_DIR})

if (WITH_TESTS)
	set(srcs
        test/NitfWriterTest.cpp
    )

    PDAL_ADD_TEST(pdal_io_nitf_writer_test
        FILES "${srcs}"
        LINK_WITH ${writer_libname})
    set(srcs
        test/NitfReaderTest.cpp
    )
    target_include_directories(pdal_io_nitf_writer_test PRIVATE
        ${PDAL_IO_DIR})

    PDAL_ADD_TEST(pdal_io_nitf_reader_test
        FILES "${srcs}"
        LINK_WITH ${reader_libname})
    target_include_directories(pdal_io_nitf_reader_test PRIVATE
        ${PDAL_IO_DIR})
endif()
