
add_definitions(-DHAVE_JSONCPP=1)
if (WIN32)
add_definitions("-DARBITER_DLL_IMPORT")
endif()

add_subdirectory(lepcc)  
set_property(TARGET lepcc PROPERTY POSITION_INDEPENDENT_CODE TRUE)


PDAL_ADD_PLUGIN(reader_libname reader i3s
    FILES
      io/i3sReader.cpp
      io/i3sReceiver.cpp
      io/SlpkExtractor.cpp
    LINK_WITH
        ${PDAL_JSONCPP_LIB_NAME}
        ${WINSOCK_LIBRARY}
)

target_link_libraries(${reader_libname} PRIVATE
    ${GDAL_LIBRARY}
    ${ZLIB_LIBRARY} 
    lepcc
)

include(${PDAL_CMAKE_DIR}/zlib.cmake)
if (ZLIB_FOUND)
    target_compile_definitions(${reader_libname} PRIVATE -DARBITER_ZLIB)
endif()

target_include_directories(${reader_libname} PRIVATE
    ${PDAL_JSONCPP_INCLUDE_DIR}
    ${PDAL_VENDOR_DIR}
    ${LEPCC_INCLUDE_DIR}
    ${PDAL_VENDOR_DIR}/eigen
    ${GDAL_INCLUDE_DIR}
)

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_io_i3s_reader_test
            FILES
                test/i3sReaderTest.cpp
            LINK_WITH
                ${reader_libname}
                ${writer_libname}
                ${PDAL_JSONCPP_LIB_NAME}
    )
    target_include_directories(pdal_io_i3s_reader_test PRIVATE
            ${PDAL_JSONCPP_INCLUDE_DIR}
            ${PDAL_VENDOR_DIR}
            ${LEPCC_INCLUDE_DIR}
            ${PDAL_VENDOR_DIR}/eigen
            ${GDAL_INCLUDE_DIR}
    )
endif()
