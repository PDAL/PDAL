
include(${PDAL_CMAKE_DIR}/zlib.cmake)

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
        ${GDAL_LIBRARY}
        ${ZLIB_LIBRARY}
        lepcc
    INCLUDES
        ${PDAL_JSONCPP_INCLUDE_DIR}
        ${PDAL_VENDOR_DIR}
        ${LEPCC_INCLUDE_DIR}
        ${PDAL_VENDOR_DIR}/eigen
        ${GDAL_INCLUDE_DIR}
)
target_compile_definitions(${i3s_reader_libname}
    PRIVATE
        ARBITER_ZLIB
        ARBITER_DLL_IMPORT
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
        ${GDAL_LIBRARY}
        ${ZLIB_LIBRARY}
        lepcc
    INCLUDES
        ${PDAL_JSONCPP_INCLUDE_DIR}
        ${PDAL_VENDOR_DIR}
        ${LEPCC_INCLUDE_DIR}
        ${PDAL_VENDOR_DIR}/eigen
        ${GDAL_INCLUDE_DIR}
)
target_compile_definitions(${slpk_reader_libname}
    PRIVATE
        ARBITER_ZLIB
        ARBITER_DLL_IMPORT
)

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_io_slpk_reader_test
            FILES
                test/SlpkReaderTest.cpp
            LINK_WITH
                ${slpk_reader_libname}
                ${PDAL_JSONCPP_LIB_NAME}
                ${GDAL_LIBRARY}
            INCLUDES
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
                INCLUDES
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

