cmake_minimum_required(VERSION 3.13)

if (STANDALONE)
    project(pdal-spz)
    find_package(PDAL REQUIRED)
    include("${PROJECT_SOURCE_DIR}/../../cmake/standalone.cmake" )
endif(STANDALONE)

include(${PDAL_CMAKE_DIR}/spz.cmake)
if (NOT PDAL_HAVE_SPZ)
    message(FATAL_ERROR "Can't find SPZ support required by SPZ plugin.")
else()
    set(SPZ_LIBRARIES spz::spz)
endif()

get_target_property(SPZ_INCLUDE_DIR spz::spz INTERFACE_INCLUDE_DIRECTORIES)

# version < 3.0.0: do nothing
if (EXISTS "${SPZ_INCLUDE_DIR}/load-spz.h")
    set(SPZ_INCLUDE_DIR "${SPZ_INCLUDE_DIR}")
# version >= 3.0.0: new include location
elseif (EXISTS "${SPZ_INCLUDE_DIR}/spz/load-spz.h")
    set(SPZ_INCLUDE_DIR "${SPZ_INCLUDE_DIR}/spz")
else()
    message(FATAL_ERROR "Could not find expected include layout for SPZ library.")
endif()

PDAL_ADD_PLUGIN(spz_reader reader spz
    FILES
        io/SpzReader.cpp
    LINK_WITH
        ${SPZ_LIBRARIES}
        ${PDAL_LIBRARIES}
    INCLUDES
        ${PDAL_VENDOR_DIR}
        ${NLOHMANN_INCLUDE_DIR}
        ${SPZ_INCLUDE_DIR}
)

PDAL_ADD_PLUGIN(spz_writer writer spz
    FILES
        io/SpzWriter.cpp
    LINK_WITH
        ${SPZ_LIBRARIES}
        ${PDAL_LIBRARIES}
    INCLUDES
        ${PDAL_VENDOR_DIR}
        ${NLOHMANN_INCLUDE_DIR}
        ${SPZ_INCLUDE_DIR}
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
            ${SPZ_INCLUDE_DIR}
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
            ${SPZ_INCLUDE_DIR}
        )
endif()
