#
# NITF plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/nitf)
include_directories(${PROJECT_SOURCE_DIR}/io/las)

find_package(Nitro 2.6 REQUIRED)
if(NITRO_FOUND)
    message(STATUS "Building with NITF support")

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

    set(deps ${NITRO_LIBRARIES})
    PDAL_ADD_PLUGIN(nitf_reader_lib_name reader nitf "${srcs}" "${incs}" "${deps}")

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

    set(deps ${NITRO_LIBRARIES})
    PDAL_ADD_PLUGIN(nitf_writer_lib_name writer nitf "${srcs}" "${incs}" "${deps}")

    if (WITH_TESTS)
	set(srcs
	    test/NitfReaderTest.cpp
	    test/NitfWriterTest.cpp
	)

	set(deps ${nitf_reader_lib_name} ${nitf_writer_lib_name})
	PDAL_ADD_TEST(nitftest FILES "${srcs}" LINK_WITH "${deps}")
    endif()
endif()
