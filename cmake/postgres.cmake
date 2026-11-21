#
# PostgreSQL configuration.
#

find_package(PostgreSQL REQUIRED)

mark_as_advanced(CLEAR POSTGRESQL_INCLUDE_DIR)
mark_as_advanced(CLEAR POSTGRESQL_LIBRARIES)
