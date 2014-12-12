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

    set(PDAL_LAS_PATH "${PROJECT_SOURCE_DIR}/io/las")

    if ((GEOTIFF_FOUND) AND (GDAL_FOUND))
        set(PDAL_GTIFF_SRCS "${PDAL_LAS_PATH}/GeotiffSupport.cpp")
        set(PDAL_GTIFF_INCS "${PDAL_LAS_PATH}/GeotiffSupport.hpp")
    endif()

    if (LASZIP_FOUND)
        set(PDAL_LASZIP_SRCS "${PDAL_LAS_PATH}/ZipPoint.cpp")
        set(PDAL_LASZIP_INCS "${PDAL_LAS_PATH}/ZipPoint.hpp")
    endif()

    set(PDAL_LAS_SRCS
        ${PDAL_GTIFF_SRCS}
        ${PDAL_LASZIP_SRCS}
        "${PROJECT_SOURCE_DIR}/src/gitsha.cpp"
	"${PDAL_LAS_PATH}/LasHeader.cpp"
	"${PDAL_LAS_PATH}/LasReader.cpp"
        "${PDAL_LAS_PATH}/SummaryData.cpp"
        "${PDAL_LAS_PATH}/VariableLengthRecord.cpp"
	"${PDAL_LAS_PATH}/LasWriter.cpp"
    )

    set(PDAL_LAS_INCS
        ${PDAL_GTIFF_INCS}
        ${PDAL_LASZIP_INCS}
        "${PROJECT_SOURCE_DIR}/include/pdal/gitsha.h"
	"${PDAL_LAS_PATH}/LasHeader.hpp"
	"${PDAL_LAS_PATH}/LasReader.hpp"
	"${PDAL_LAS_PATH}/SummaryData.hpp"
	"${PDAL_LAS_PATH}/VariableLengthRecord.hpp"
	"${PDAL_LAS_PATH}/LasWriter.hpp"
    )

    #
    # NITF Reader
    #
    set(srcs
        ${PDAL_LAS_SRCS}
	io/NitfFile.cpp
	io/MetadataReader.cpp
	io/tre_plugins.cpp
	io/NitfReader.cpp
    )

    set(incs
        ${PDAL_LAS_INCS}
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
        ${PDAL_LAS_SRCS}
	io/NitfWriter.cpp
	io/tre_plugins.cpp
    )

    set(incs
        ${PDAL_LAS_INCS}
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
	PDAL_ADD_TEST(nitftest "${srcs}" "${deps}")
    endif()
endif()
