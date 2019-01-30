#
# TileDB support
#
find_package(TileDB QUIET 1.4.1 REQUIRED)
set_package_properties(TileDB PROPERTIES
        TYPE OPTIONAL
        URL "https://www.tiledb.io"
        PURPOSE "TileDB support")

if(TILEDB_FOUND)
    set(CMAKE_REQUIRED_LIBRARIES "${TILEDB_LIBRARIES}")
    include_directories(${TILEDB_INCLUDE_DIRS})
    mark_as_advanced(CLEAR TILEDB_INCLUDE_DIRS)
    mark_as_advanced(CLEAR TILEDB_LIBRARIES)
endif()
