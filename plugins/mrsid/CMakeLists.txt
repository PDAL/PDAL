#
# MrSID plugin CMake configuration
#

set(MRSID_ROOT "/Users/hobu/installs/mrsid/Lidar_DSDK" CACHE STRING "Root directory of MrSID install")
find_package(MrSID)
if (MRSID_FOUND)
    PDAL_ADD_PLUGIN(reader_libname reader mrsid
        FILES
            io/MrsidReader.cpp
        LINK_WITH
            ${MRSID_LIBRARY}
        INCLUDES
            ${PDAL_IO_DIR}
            ${NLOHMANN_INCLUDE_DIR}
            ${MRSID_INCLUDE_DIR}
    )
    target_compile_definitions(${reader_libname} PRIVATE -DHAVE_MRSID=1)

    if (WITH_TESTS)
        PDAL_ADD_TEST(mrsidtest
            FILES
                test/MrsidTest.cpp
            LINK_WITH
                ${reader_libname}
            INCLUDES
                ${PDAL_IO_DIR}
                ${NLOHMANN_INCLUDE_DIR}
                ${MRSID_INCLUDE_DIR}

        )
    endif()
else()
    message(STATUS "Building without MrSID support")
endif()
