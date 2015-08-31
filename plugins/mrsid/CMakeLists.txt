#
# MrSID plugin CMake configuration
#

set(MRSID_ROOT "/Users/hobu/installs/mrsid/Lidar_DSDK" CACHE STRING "Root directory of MrSID install")
find_package(MrSID)
if(MRSID_FOUND)
    message(STATUS "Building with MrSID support")

    include_directories(${MRSID_INCLUDE_DIR})
    add_definitions(-DHAVE_MRSID=1)

    set(srcs io/MrsidReader.cpp)
    set(incs io/MrsidReader.hpp)

    PDAL_ADD_PLUGIN(libname reader mrsid
        FILES "${srcs}" "${incs}"
        LINK_WITH ${MRSID_LIBRARY})
    if (WITH_TESTS)
        PDAL_ADD_TEST(mrsidtest
            FILES test/MrsidTest.cpp
            LINK_WITH ${libname})
    endif()
else()
    message(STATUS "Building without MrSID support")
endif()
