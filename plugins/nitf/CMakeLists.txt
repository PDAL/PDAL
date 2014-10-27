#
# NITF plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/nitf)

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

    set(PDAL_LAS_PATH drivers/las)
    set(PDAL_LAS_HEADERS "${PROJECT_SOURCE_DIR}/include/pdal/${PDAL_LAS_PATH}")
    set(PDAL_LAS_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_LAS_PATH}")

    if ((GEOTIFF_FOUND) AND (GDAL_FOUND))
        set(PDAL_GTIFF_SRCS "${PDAL_LAS_SRC}/GeotiffSupport.cpp")
	set(PDAL_GTIFF_INCS "${PDAL_LAS_SRC}/GeotiffSupport.hpp")
    endif()

    if (LASZIP_FOUND)
        set(PDAL_LASZIP_SRCS "${PDAL_LAS_SRC}/ZipPoint.cpp")
	set(PDAL_LASZIP_INCS "${PDAL_LAS_HEADERS}/ZipPoint.hpp")
    endif()

    set(PDAL_LAS_SRCS
        ${PDAL_GTIFF_SRCS}
        ${PDAL_LASZIP_SRCS}
        "${PDAL_LAS_SRC}/Header.cpp"
        "${PDAL_LAS_SRC}/Reader.cpp"
        "${PDAL_LAS_SRC}/SummaryData.cpp"
        "${PDAL_LAS_SRC}/VariableLengthRecord.cpp"
        "${PDAL_LAS_SRC}/Writer.cpp"
    )

    set(PDAL_LAS_INCS
        ${PDAL_GTIFF_INCS}
        ${PDAL_LASZIP_INCS}
        "${PDAL_LAS_HEADERS}/Header.hpp"
	"${PDAL_LAS_HEADERS}/Reader.hpp"
	"${PDAL_LAS_HEADERS}/SummaryData.hpp"
	"${PDAL_LAS_HEADERS}/VariableLengthRecord.hpp"
	"${PDAL_LAS_HEADERS}/Writer.hpp"
    )

    #
    # NITF Reader
    #
    set(srcs
        ${PDAL_LAS_SRCS}
	drivers/NitfFile.cpp
	drivers/MetadataReader.cpp
	drivers/tre_plugins.cpp
	drivers/NitfReader.cpp
    )

    set(incs
        ${PDAL_LAS_INCS}
	drivers/NitfFile.hp
	drivers/MetadataReader.hpp
	drivers/tre_plugins.hpp
	drivers/NitfReader.hpp
    )

    set(NITF_READER_LIB_NAME pdal_plugin_reader_nitf)
    PDAL_ADD_PLUGIN(${NITF_READER_LIB_NAME} reader nitf ${srcs} ${incs})
    target_link_libraries(${NITF_READER_LIB_NAME} ${PDAL_LINKAGE} ${NITRO_LIBRARIES})

    #
    # NITF Writer
    #
    set(srcs
        ${PDAL_LAS_SRCS}
	drivers/NitfWriter.cpp
	drivers/tre_plugins.cpp
    )

    set(incs
        ${PDAL_LAS_INCS}
	drivers/NitfWriter.hpp
	drivers/tre_plugins.hpp
    )

    set(NITF_WRITER_LIB_NAME pdal_plugin_writer_nitf)
    PDAL_ADD_PLUGIN(${NITF_WRITER_LIB_NAME} writer nitf ${srcs} ${incs})
    target_link_libraries(${NITF_WRITER_LIB_NAME} ${PDAL_LINKAGE} ${NITRO_LIBRARIES})

    set(srcs
        test/NitfReaderTest.cpp
        test/NitfWriterTest.cpp
    )

    PDAL_ADD_TEST(nitftest ${srcs})
else()
    message(STATUS "Building without NITF support")
endif()
