#
# CachedEpt plugin CMake configuration
#
include(${PDAL_CMAKE_DIR}/tbb.cmake)
if (NOT TBB_FOUND)
    message(FATAL_ERROR "Can't find TBB support required.")
endif()

if (WIN32)
    add_definitions(-DNOMINMAX)
endif()

#
# EPT LRU-Cached Reader
#
PDAL_ADD_PLUGIN(reader_libname reader cachedept
    FILES
        io/CachedTileContents.cpp
        io/CachedEptReader.cpp
        io/private/ept/Addon.cpp
        io/private/ept/Connector.cpp
        io/private/ept/EptInfo.cpp
        io/private/ept/FixedPointLayout.cpp
        io/private/ept/TileContents.cpp
    LINK_WITH
        ${TBB_LIBRARIES}
    INCLUDES
        ${TBB_INCLUDE_DIR}
        ${NLOHMANN_INCLUDE_DIR}
        ${PDAL_VENDOR_DIR}
)
