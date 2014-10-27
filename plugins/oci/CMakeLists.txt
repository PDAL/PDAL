#
# Oracle OCI plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugin/oci)

find_package(Oracle REQUIRED)
if(ORACLE_FOUND)
    message(STATUS "Building with Oracle support")

    include_directories(${ORACLE_INCLUDE_DIR})

    #
    # OCI Reader
    #
    set(srcs
	drivers/OciCommon.cpp
	drivers/OciReader.cpp
	drivers/OciWrapper.cpp
    )

    set(incs
	drivers/OciCommon.hpp
	drivers/OciReader.hpp
	drivers/OciWrapper.h
    )

    set(OCI_READER_LIB_NAME pdal_plugin_reader_oci)
    PDAL_ADD_PLUGIN(${OCI_READER_LIB_NAME} reader oci ${srcs} ${incs})
    target_link_libraries(${OCI_READER_LIB_NAME} ${PDAL_LINKAGE} ${ORACLE_LIBRARY})

    #
    # OCI Writer
    #
    set(srcs
	drivers/OciCommon.cpp
	drivers/OciWrapper.cpp
	drivers/OciWriter.cpp
    )

    set(incs
	drivers/OciCommon.hpp
	drivers/OciWrapper.h
	drivers/OciWriter.hpp
    )

    set(OCI_WRITER_LIB_NAME pdal_plugin_writer_oci)
    PDAL_ADD_PLUGIN(${OCI_WRITER_LIB_NAME} writer oci ${srcs} ${incs})
    target_link_libraries(${OCI_WRITER_LIB_NAME} ${PDAL_LINKAGE} ${ORACLE_LIBRARY})
else()
    message(STATUS "Building without Oracle support")
endif()
