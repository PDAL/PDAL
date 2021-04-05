#
# TileDB support
#
set(FPHSA_NAME_MISMATCHED 1) # Suppress warnings, see https://cmake.org/cmake/help/v3.17/module/FindPackageHandleStandardArgs.html
find_package(TileDB QUIET 1.4.1 REQUIRED)
set(FPHSA_NAME_MISMATCHED 0)

set_package_properties(TileDB PROPERTIES
        TYPE OPTIONAL
        URL "https://www.tiledb.io"
        PURPOSE "TileDB support")

mark_as_advanced(CLEAR TILEDB_INCLUDE_DIRS)
mark_as_advanced(CLEAR TILEDB_LIBRARIES)
