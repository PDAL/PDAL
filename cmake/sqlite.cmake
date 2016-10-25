#
# SQLite cmake configuration
#

find_package(SQLite3 QUIET REQUIRED)
if(NOT "${SQLITE3_HAS_LOAD_EXTENSION}")
    message(FATAL_ERROR "SQLite3 compiled without load extension, which is required. Please re-compile SQLite3 with load extension or ensure CMake is pointing at the correct SQLite3 installation.")
endif()
mark_as_advanced(CLEAR SQLITE3_INCLUDE_DIR)
mark_as_advanced(CLEAR SQLITE3_LIBRARY)
include_directories(${SQLITE3_INCLUDE_DIR})
