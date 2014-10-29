#
# Icebridge plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/icebridge)

set (HDF5_FIND_COMPONENTS "CXX")
find_package(HDF5 COMPONENTS CXX REQUIRED)
if (HDF5_FOUND)
    message(STATUS "Building with Icebridge support")

    include_directories(${HDF5_INCLUDE_DIR})
    add_definitions(-DHAVE_HDF5=1)

    #
    # Icebridge Reader
    #
    set(srcs
	drivers/IcebridgeReader.cpp
	drivers/Hdf5Handler.cpp
    )

    set(incs
	drivers/IcebridgeReader.hpp
	drivers/Hdf5Handler.hpp
    )

    set(ICEBRIDGE_LIB_NAME pdal_plugin_reader_icebridge)
    PDAL_ADD_PLUGIN(${ICEBRIDGE_LIB_NAME} reader icebridge ${srcs} ${incs})
    target_link_libraries(${ICEBRIDGE_LIB_NAME} ${PDAL_LINKAGE} ${HDF5_LIBRARIES})

    #
    # Icebridge tests
    #
    set(srcs
        test/IcebridgeReaderTest.cpp
    )

    PDAL_ADD_TEST(icetest ${srcs})
else()
    message(STATUS "Building without Icebridge support")
endif()
