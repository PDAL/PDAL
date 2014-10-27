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
        drivers/MrsidReader.cpp
    )

    set(incs
        drivers/MrsidReader.hpp
    )

    set(MRSID_LIB_NAME pdal_plugin_reader_mrsid)
    PDAL_ADD_PLUGIN(${MRSID_LIB_NAME} reader mrsid ${srcs} ${incs})
    target_link_libraries(${MRSID_LIB_NAME} ${PDAL_LINKAGE} ${MRSID_LIBRARY})
else()
    message(STATUS "Building without MrSID support")
endif()
