
PDAL_ADD_PLUGIN(libname reader stac
    FILES
        io/stacReader.cpp
    LINK_WITH
        ${STAC_LIBRARIES}
    INCLUDES
        ${ROOT_DIR}
        ${NLOHMANN_INCLUDE_DIR}
)

# if (WITH_TESTS)