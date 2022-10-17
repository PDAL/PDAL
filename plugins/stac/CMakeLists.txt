
PDAL_ADD_PLUGIN(stac_reader_libname reader stac
    FILES
        io/stacReader.cpp
    LINK_WITH
        ${STAC_LIBRARIES}
    INCLUDES
        ${ROOT_DIR}
        ${PDAL_VENDOR_DIR}
        ${NLOHMANN_INCLUDE_DIR}
)
target_compile_definitions(${stac_reader_libname}
    PRIVATE
        ARBITER_DLL_IMPORT
)

# if (WITH_TESTS)