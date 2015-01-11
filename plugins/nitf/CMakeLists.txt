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
    io/NitfFile.cpp
    io/MetadataReader.cpp
    io/tre_plugins.cpp
    io/NitfReader.cpp
)

set(incs
    io/NitfFile.hpp
    io/MetadataReader.hpp
    io/tre_plugins.hpp
    io/NitfReader.hpp
)

PDAL_ADD_PLUGIN(reader_libname reader nitf
    FILES "${srcs}" "${incs}"
    LINK_WITH ${NITRO_LIBRARIES})

#
# NITF Writer
#
set(srcs
    io/NitfWriter.cpp
    io/tre_plugins.cpp
)

set(incs
    io/NitfWriter.hpp
    io/tre_plugins.hpp
)

PDAL_ADD_PLUGIN(writer_libname writer nitf
    FILES "${srcs}" "${incs}"
    LINK_WITH ${NITRO_LIBRARIES})

if (WITH_TESTS)
	set(srcs
        test/NitfReaderTest.cpp
        test/NitfWriterTest.cpp
    )

    PDAL_ADD_TEST(nitftest
        FILES "${srcs}"
        LINK_WITH ${reader_libname} ${writer_libname})
endif()
