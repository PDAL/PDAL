cmake_minimum_required(VERSION 3.13)

if (STANDALONE)
    project(pdal-spz)
    find_package(PDAL REQUIRED)
    include("${PROJECT_SOURCE_DIR}/../../cmake/standalone.cmake" )
endif(STANDALONE)

add_subdirectory(spz)

PDAL_ADD_PLUGIN(spz_reader reader spz
    FILES
        io/SpzReader.cpp
    LINK_WITH
        libspz
        ${PDAL_LIBRARIES}
    INCLUDES
        ${PDAL_VENDOR_DIR}
        ${NLOHMANN_INCLUDE_DIR}
        spz/src/cc
)

PDAL_ADD_PLUGIN(spz_writer writer spz
    FILES
        io/SpzWriter.cpp
    LINK_WITH
        libspz
        ${PDAL_LIBRARIES}
    INCLUDES
        ${PDAL_VENDOR_DIR}
        ${NLOHMANN_INCLUDE_DIR}
        spz/src/cc
)

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_io_spz_reader_test
        FILES
            test/SpzReaderTest.cpp
        LINK_WITH
            ${PDAL_LIBRARIES}
            ${spz_reader}
        INCLUDES
            ${PROJECT_SOURCE_DIR}/plugins/spz/io
            spz/src/cc
    )

    PDAL_ADD_TEST(pdal_io_spz_writer_test
        FILES
            test/SpzWriterTest.cpp
        LINK_WITH
            ${PDAL_LIBRARIES}
            ${spz_writer}
            ${spz_reader}
        INCLUDES
            ${PROJECT_SOURCE_DIR}/plugins/spz/io
            spz/src/cc
        )
endif()