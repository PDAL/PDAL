#
# SQLite cmake configuration
#

find_package(SQLite3 QUIET REQUIRED)
mark_as_advanced(CLEAR SQLITE3_INCLUDE_DIR)
mark_as_advanced(CLEAR SQLITE3_LIBRARY)
include_directories(${SQLITE3_INCLUDE_DIR})
