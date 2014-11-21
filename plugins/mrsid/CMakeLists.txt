#
# MrSID plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/mrsid)

set(MRSID_ROOT "/Users/hobu/installs/mrsid/Lidar_DSDK" CACHE STRING "Root directory of MrSID install")
find_package(MrSID)
if(MRSID_FOUND)
    message(STATUS "Building with MrSID support")

    include_directories(${MRSID_INCLUDE_DIR})
    add_definitions(-DHAVE_MRSID=1)

    #
    # MrSID Reader
    #
    set(srcs
        io/MrsidReader.cpp
    )

    set(incs
        io/MrsidReader.hpp
    )

    set(deps ${MRSID_LIBRARY})
    PDAL_ADD_PLUGIN(mrsid_reader_lib_name reader mrsid "${srcs}" "${incs}" "${deps}")
else()
    message(STATUS "Building without MrSID support")
endif()
