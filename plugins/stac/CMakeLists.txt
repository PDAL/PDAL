# find_package(nlohmann_json_schema_validator REQUIRED)
PDAL_ADD_PLUGIN(stac_reader_libname reader stac
    FILES
        io/stacReader.cpp
    LINK_WITH
        ${STAC_LIBRARIES}
        ${JSON_SCHEMA_LIB_NAME}
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