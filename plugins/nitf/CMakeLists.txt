cmake_minimum_required(VERSION 3.13)

if (STANDALONE)
    project(pdal-nitf)
    find_package(PDAL REQUIRED)
    include("${PROJECT_SOURCE_DIR}/../../cmake/standalone.cmake" )
    include(${PDAL_CMAKE_DIR}/arbiter.cmake)
    include(${PDAL_CMAKE_DIR}/nlohmann.cmake)
    add_subdirectory("${PROJECT_SOURCE_DIR}/../../vendor/arbiter" "vendor/arbiter")

endif(STANDALONE)

#
# NITF plugin CMake configuration
#

include(${PDAL_CMAKE_DIR}/nitro.cmake)
include(${PDAL_CMAKE_DIR}/arbiter.cmake)
if (NOT NITRO_FOUND)
    message(FATAL_ERROR "Can't find NITRO support required by NITF.")
endif()

PDAL_ADD_PLUGIN(reader_libname reader nitf
    FILES
        io/NitfFileReader.cpp
        io/MetadataReader.cpp
        io/tre_plugins.cpp
        io/NitfReader.cpp
    INCLUDES
        ${GDAL_INCLUDE_DIR}
        ${PDAL_VENDOR_DIR}
        ${NLOHMANN_INCLUDE_DIR}
    LINK_WITH
        ${NITRO_LIBRARIES}
        ${PDAL_ARBITER_LIB_NAME}
        ${PDAL_LIBRARIES}
)
if (WIN32)
    target_compile_definitions(${reader_libname}
        PRIVATE
            WIN32
            _HAS_AUTO_PTR_ETC=1
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
        ${PDAL_LIBRARIES}
        ${PDAL_ARBITER_LIB_NAME}
    INCLUDES
        ${GDAL_INCLUDE_DIR}
)
if (WIN32)
    target_compile_definitions(${writer_libname}
        PRIVATE
            WIN32
            _HAS_AUTO_PTR_ETC=1
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
