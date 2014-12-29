#
# NITF plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/nitf)
include_directories(${PROJECT_SOURCE_DIR}/io/las)

find_package(Nitro QUIET 2.6)

set_package_properties(Nitro PROPERTIES DESCRIPTION "Nitro" URL "http://foo"
    TYPE OPTIONAL PURPOSE "Enables NITF reader/writer")

if(NITRO_FOUND)
    include_directories(${NITRO_INCLUDE_DIR})
    include_directories("${NITRO_INCLUDE_DIR}/nitro/c++")
    include_directories("${NITRO_INCLUDE_DIR}/nitro/c")
    add_definitions("-D_REENTRANT")
    if (WIN32)
        add_definitions("-DSIZEOF_SIZE_T=4")
        add_definitions("-DIMPORT_NITRO_API")
    else()
        add_definitions("-D__POSIX")
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
endif()
