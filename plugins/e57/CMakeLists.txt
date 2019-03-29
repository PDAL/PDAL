include(FindXercesC)
find_package(XercesC REQUIRED)

add_subdirectory(libE57Format)

PDAL_ADD_PLUGIN(e57plugin_reader reader e57
        FILES
            io/e57reader.cpp
            io/scan.cpp
            io/utils.cpp
            io/uuid.cpp
        LINK_WITH
            E57Format
            ${XercesC_LIBRARY}
)

target_include_directories(${e57plugin_reader} PRIVATE "libE57Format/include")

PDAL_ADD_PLUGIN(e57plugin_writer writer e57
        FILES
            io/e57writer.cpp
            io/scan.cpp
            io/utils.cpp
            io/uuid.cpp
        LINK_WITH
            E57Format
            ${XercesC_LIBRARY}
        )

target_include_directories(${e57plugin_writer} PRIVATE "libE57Format/include")

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_io_e57_reader_test
            FILES
                test/reader-test.cpp
                test/scan-test.cpp
                test/uuid-test.cpp
                test/utils-test.cpp
            LINK_WITH
                ${e57plugin_reader}
                E57Format
            )
    target_include_directories(pdal_io_e57_reader_test PRIVATE "libE57Format/include")

    PDAL_ADD_TEST(pdal_io_e57_writer_test
            FILES
                test/writer-test.cpp
            LINK_WITH
                ${e57plugin_reader}
                ${e57plugin_writer}
                E57Format
            )
    target_include_directories(pdal_io_e57_writer_test PRIVATE "libE57Format/include")
endif()
