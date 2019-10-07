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

if(WITH_TESTS)
    PDAL_ADD_TEST(pdal_filters_voxeldownsize_test
        FILES
            test/VoxelDownsizeFilterTest.cpp
        LINK_WITH
            ${filter_libname}
        INCLUDES
			${CMAKE_CURRENT_LIST_DIR}
			${PDAL_VENDOR_DIR}
			${PDAL_VENDOR_DIR}/nlohmann
        )
endif(WITH_TESTS)
