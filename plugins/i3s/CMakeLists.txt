include(${PDAL_CMAKE_DIR}/zlib.cmake)

add_subdirectory(lepcc)

PDAL_ADD_PLUGIN(i3s_reader_libname reader i3s
    FILES
      io/I3SReader.cpp
      io/EsriUtil.cpp
      io/EsriReader.cpp
      io/Obb.cpp
    LINK_WITH
        ${WINSOCK_LIBRARY}
        ${ZLIB_LIBRARY}
        lepcc
    INCLUDES
        ${NLOHMANN_INCLUDE_DIR}
        ${PDAL_VENDOR_DIR}
        ${PDAL_VENDOR_DIR}/eigen
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
    LINK_WITH
        ${WINSOCK_LIBRARY}
        ${ZLIB_LIBRARY}
        lepcc
    INCLUDES
        ${NLOHMANN_INCLUDE_DIR}
        ${PDAL_VENDOR_DIR}
        ${PDAL_VENDOR_DIR}/eigen
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
                io/Obb.cpp
            INCLUDES
                ${NLOHMANN_INCLUDE_DIR}
                ${PDAL_VENDOR_DIR}/eigen
    )

    if (BUILD_I3S_TESTS)
        PDAL_ADD_TEST(pdal_io_i3s_reader_test
                FILES
                    test/i3sReaderTest.cpp
        )
    endif()
endif()

