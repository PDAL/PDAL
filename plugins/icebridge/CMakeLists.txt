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
	io/IcebridgeReader.cpp
	io/Hdf5Handler.cpp
    )

    set(incs
	io/IcebridgeReader.hpp
	io/Hdf5Handler.hpp
    )

    set(deps ${HDF5_LIBRARIES})
    PDAL_ADD_PLUGIN(icebridge_reader_lib_name reader icebridge "${srcs}" "${incs}" "${deps}")

    if (WITH_TESTS)
        PDAL_ADD_TEST(icetest FILES test/IcebridgeReaderTest.cpp LINK_WITH ${icebridge_reader_lib_name})
    endif()
else()
    message(STATUS "Building without Icebridge support")
endif()
