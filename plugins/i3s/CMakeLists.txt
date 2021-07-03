include(${PDAL_CMAKE_DIR}/zlib.cmake)

if (NOT ZLIB_FOUND)
    message(FATAL_ERROR "ZLIB can't be found. Can't build SLPK/I3S without ZLIB support")
endif()

add_subdirectory(lepcc)

PDAL_ADD_PLUGIN(i3s_reader_libname reader i3s
    FILES
        io/I3SReader.cpp
        io/EsriUtil.cpp
        io/EsriReader.cpp
        io/Obb.cpp
        io/PageManager.cpp
        ${PDAL_SRC_DIR}/private/MathUtils.cpp
    LINK_WITH
        ${WINSOCK_LIBRARY}
        lepcc
    INCLUDES
        ${NLOHMANN_INCLUDE_DIR}
        ${PDAL_VENDOR_DIR}
        ${PDAL_VENDOR_DIR}/eigen
)
target_compile_definitions(${i3s_reader_libname}
    PRIVATE
        ARBITER_DLL_IMPORT
)

PDAL_ADD_PLUGIN(slpk_reader_libname reader slpk
    FILES
        io/SlpkReader.cpp
        io/EsriUtil.cpp
        io/EsriReader.cpp
        io/Obb.cpp
        io/PageManager.cpp
        ${PDAL_SRC_DIR}/private/MathUtils.cpp
    LINK_WITH
        ${WINSOCK_LIBRARY}
        lepcc
    INCLUDES
        ${NLOHMANN_INCLUDE_DIR}
        ${PDAL_VENDOR_DIR}
        ${PDAL_VENDOR_DIR}/eigen
)
target_compile_definitions(${slpk_reader_libname}
    PRIVATE
        ARBITER_DLL_IMPORT
)

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_io_slpk_reader_test
        FILES
            test/SlpkReaderTest.cpp
    )

    PDAL_ADD_TEST(pdal_i3s_obb_test
        FILES
            test/ObbTest.cpp
            io/Obb.cpp
            ${PDAL_SRC_DIR}/private/MathUtils.cpp
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

