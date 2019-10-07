include(${PDAL_CMAKE_DIR}/leveldb.cmake)
PDAL_ADD_PLUGIN(filter_libname filter voxelownsize
    FILES
        filter/VoxelDownsizeFilter.cpp
    LINK_WITH
        leveldb
    INCLUDES
        ${PDAL_VENDOR_DIR}
        ${PDAL_VENDOR_DIR}/nlohmann
)
