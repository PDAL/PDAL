#
# PgPointCloud plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/pgpointcloud)

find_package(PostgreSQL REQUIRED)
if (POSTGRESQL_FOUND)
    message(STATUS "Building with PgPointCloud support")

    mark_as_advanced(CLEAR POSTGRESQL_INCLUDE_DIR)
    mark_as_advanced(CLEAR POSTGRESQL_LIBRARIES)
    include_directories(${POSTGRESQL_INCLUDE_DIR})

    #
    # PgPointCloud Reader
    #
    set(srcs
	drivers/PgReader.cpp
    )

    set(incs
	drivers/PgCommon.hpp
	drivers/PgReader.hpp
    )

    set(PGPOINTCLOUD_READER_LIB_NAME pdal_plugin_reader_pgpointcloud)
    PDAL_ADD_PLUGIN(${PGPOINTCLOUD_READER_LIB_NAME} reader pgpointcloud ${srcs} ${incs})
    target_link_libraries(${PGPOINTCLOUD_READER_LIB_NAME} ${PDAL_LINKAGE} ${POSTGRESQL_LIBRARIES})

    #
    # PgPointCloud Writer
    #
    set(srcs
	drivers/PgWriter.cpp
    )

    set(incs
	drivers/PgCommon.hpp
	drivers/PgWriter.hpp
    )

    set(PGPOINTCLOUD_WRITER_LIB_NAME pdal_plugin_writer_pgpointcloud)
    PDAL_ADD_PLUGIN(${PGPOINTCLOUD_WRITER_LIB_NAME} writer pgpointcloud ${srcs} ${incs})
    target_link_libraries(${PGPOINTCLOUD_WRITER_LIB_NAME} ${PDAL_LINKAGE} ${POSTGRESQL_LIBRARIES})
else()
    message(STATUS "Building without PgPointCloud support")
endif()
