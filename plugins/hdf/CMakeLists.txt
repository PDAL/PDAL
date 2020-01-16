#
# Icebridge plugin CMake configuration
#

include (${PDAL_CMAKE_DIR}/hdf5.cmake)


if (NOT PDAL_HAVE_HDF5)
    message(FATAL "HDF5 not found but is required for Icebridge.")
else()
    PDAL_ADD_PLUGIN(libname reader hdf
        FILES
            io/HdfReader.cpp
            io/Hdf5Handler.cpp
        LINK_WITH
            ${HDF5_LIBRARIES}
        INCLUDES
            ${ROOT_DIR}
            ${LIBXML2_INCLUDE_DIR}
    )

endif()
