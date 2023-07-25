#
# TileDB support
#
find_package(TileDB QUIET 2.15.0 REQUIRED)

set_package_properties(TileDB PROPERTIES
        TYPE OPTIONAL
        URL "https://www.tiledb.com"
        PURPOSE "TileDB support")

mark_as_advanced(CLEAR TileDB_INCLUDE_DIRS)
mark_as_advanced(CLEAR TileDB_LIBRARIES)
