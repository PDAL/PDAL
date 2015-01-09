#
# Icebridge plugin CMake configuration
#

include (${PDAL_CMAKE_DIR}/hdf5.cmake)

if (NOT PDAL_HAVE_HDF5)
    message(FATAL "HDF5 not found but is required for Icebridge.")
else()
    set(srcs
        io/IcebridgeReader.cpp
        io/Hdf5Handler.cpp
    )

    set(incs
        io/IcebridgeReader.hpp
        io/Hdf5Handler.hpp
    )

    PDAL_ADD_PLUGIN(libname reader icebridge
        FILES "${srcs}" "${incs}"
        LINK_WITH ${HDF5_LIBRARIES})

    if (WITH_TESTS)
        PDAL_ADD_TEST(icetest
            FILES test/IcebridgeReaderTest.cpp
            LINK_WITH ${libname})
    endif()
endif()
