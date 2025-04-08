cmake_minimum_required(VERSION 3.13)

if (STANDALONE)
    find_package(PDAL REQUIRED)
    include("${PROJECT_SOURCE_DIR}/../../cmake/standalone.cmake" )
    include("${PROJECT_SOURCE_DIR}/../../cmake/nlohmann.cmake" )
endif(STANDALONE)


add_subdirectory(libE57Format)

PDAL_ADD_PLUGIN(e57plugin_reader reader e57
    FILES
        io/E57Reader.cpp
        io/Scan.cpp
        io/Utils.cpp
    INCLUDES
        ${PDAL_VENDOR_DIR}
    LINK_WITH
        E57Format
        nlohmann_json::nlohmann_json
        ${PDAL_LIBRARIES}
    INCLUDES
        libE57Format/include
)
target_compile_definitions(${e57plugin_reader}
    PRIVATE
        ARBITER_ZLIB
        ARBITER_DLL_IMPORT
)

PDAL_ADD_PLUGIN(e57plugin_writer writer e57
    FILES
        io/E57Writer.cpp
        io/Utils.cpp
        io/Uuid.cpp
    LINK_WITH
        E57Format
        ${PDAL_LIBRARIES}
        nlohmann_json::nlohmann_json
    INCLUDES
        libE57Format/include
)

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_io_e57_read_test
        FILES
            test/E57ReaderTest.cpp
            test/E57UtilsTest.cpp
            test/ScanTest.cpp
        LINK_WITH
            ${e57plugin_reader}
            E57Format
            nlohmann_json::nlohmann_json
        INCLUDES
            libE57Format/include
    )

    PDAL_ADD_TEST(pdal_io_e57_write_test
        FILES
            test/E57WriterTest.cpp
            test/E57UuidTest.cpp
        LINK_WITH
            ${e57plugin_reader}
            ${e57plugin_writer}
            E57Format
            nlohmann_json::nlohmann_json
        INCLUDES
            libE57Format/include
    )
endif()
