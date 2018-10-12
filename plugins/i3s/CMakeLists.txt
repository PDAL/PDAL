
add_subdirectory(lepcc)

PDAL_ADD_PLUGIN(i3s_reader_libname reader i3s
    FILES
      io/I3SReader.cpp
      io/EsriUtil.cpp
      io/SlpkExtractor.cpp
      io/EsriReader.cpp
    LINK_WITH
        ${PDAL_JSONCPP_LIB_NAME}
        ${WINSOCK_LIBRARY}
)

PDAL_ADD_PLUGIN(slpk_reader_libname reader slpk
    FILES
        io/SlpkReader.cpp
        io/EsriUtil.cpp
        io/EsriReader.cpp
        io/SlpkExtractor.cpp
    LINK_WITH
        ${PDAL_JSONCPP_LIB_NAME}
        ${WINSOCK_LIBRARY}
)


target_link_libraries(${i3s_reader_libname} PRIVATE
    ${GDAL_LIBRARY}
    ${ZLIB_LIBRARY}
    lepcc
)

target_link_libraries(${slpk_reader_libname} PRIVATE
    ${GDAL_LIBRARY}
    ${ZLIB_LIBRARY}
    lepcc
)

include(${PDAL_CMAKE_DIR}/zlib.cmake)
target_compile_definitions(${i3s_reader_libname}
    PRIVATE
        ARBITER_ZLIB
        ARBITER_DLL_IMPORT
)
target_compile_definitions(${slpk_reader_libname}
    PRIVATE
        ARBITER_ZLIB
        ARBITER_DLL_IMPORT
)

target_include_directories(${i3s_reader_libname} PRIVATE
    ${PDAL_JSONCPP_INCLUDE_DIR}
    ${PDAL_VENDOR_DIR}
    ${LEPCC_INCLUDE_DIR}
    ${PDAL_VENDOR_DIR}/eigen
    ${GDAL_INCLUDE_DIR}
)

target_include_directories(${slpk_reader_libname} PRIVATE
    ${PDAL_JSONCPP_INCLUDE_DIR}
    ${PDAL_VENDOR_DIR}
    ${LEPCC_INCLUDE_DIR}
    ${PDAL_VENDOR_DIR}/eigen
    ${GDAL_INCLUDE_DIR}
)

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_io_slpk_reader_test
            FILES
                test/SlpkReaderTest.cpp
            LINK_WITH
                ${slpk_reader_libname}
                ${PDAL_JSONCPP_LIB_NAME}
                ${GDAL_LIBRARY}
    )
    target_include_directories(pdal_io_slpk_reader_test PRIVATE
            ${PDAL_JSONCPP_INCLUDE_DIR}
            ${PDAL_VENDOR_DIR}
            ${LEPCC_INCLUDE_DIR}
            ${PDAL_VENDOR_DIR}/eigen
            ${GDAL_INCLUDE_DIR}
    )
    target_compile_definitions(pdal_io_slpk_reader_test
        PRIVATE
            ARBITER_ZLIB
            ARBITER_DLL_IMPORT
    )

    if (BUILD_I3S_TESTS)
        PDAL_ADD_TEST(pdal_io_i3s_reader_test
                FILES
                    test/i3sReaderTest.cpp
                LINK_WITH
                    ${i3s_reader_libname}
                    ${PDAL_JSONCPP_LIB_NAME}
                    ${GDAL_LIBRARY}
        )
        target_include_directories(pdal_io_i3s_reader_test PRIVATE
                ${PDAL_JSONCPP_INCLUDE_DIR}
                ${PDAL_VENDOR_DIR}
                ${LEPCC_INCLUDE_DIR}
                ${PDAL_VENDOR_DIR}/eigen
                ${GDAL_INCLUDE_DIR}
        )
        target_compile_definitions(pdal_io_i3s_reader_test
            PRIVATE
                ARBITER_ZLIB
                ARBITER_DLL_IMPORT
        )
    endif()
endif()

