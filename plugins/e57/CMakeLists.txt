include(FindXercesC)
find_package(XercesC REQUIRED)

add_subdirectory(libE57Format)

PDAL_ADD_PLUGIN(e57plugin_reader reader e57
        FILES
            io/E57Reader.cpp
            io/Scan.cpp
            io/Utils.cpp
        INCLUDES
            ${NLOHMANN_INCLUDE_DIR}
            ${PDAL_VENDOR_DIR}
        LINK_WITH
            E57Format
            ${XercesC_LIBRARY}
)

target_include_directories(${e57plugin_reader} PRIVATE "libE57Format/include")
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
            ${XercesC_LIBRARY}
        )

target_include_directories(${e57plugin_writer} PRIVATE "libE57Format/include")

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_io_e57_read_test
            FILES
                test/E57ReaderTest.cpp
                test/ScanTest.cpp
                test/E57UtilsTest.cpp
            LINK_WITH
                ${e57plugin_reader}
                E57Format
                ${XercesC_LIBRARY}
            )
    target_include_directories(pdal_io_e57_read_test PRIVATE "libE57Format/include")

    PDAL_ADD_TEST(pdal_io_e57_write_test
        FILES
            test/E57WriterTest.cpp
            test/E57UuidTest.cpp
        LINK_WITH
            ${e57plugin_reader}
            ${e57plugin_writer}
            E57Format
            ${XercesC_LIBRARY}
            )
    target_include_directories(pdal_io_e57_write_test PRIVATE "libE57Format/include")
endif()
