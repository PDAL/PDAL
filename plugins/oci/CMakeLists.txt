#
# Oracle OCI plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/oci)

find_package(Oracle)
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

    set(deps ${ORACLE_LIBRARY})
    PDAL_ADD_PLUGIN(oci_reader_lib_name reader oci "${srcs}" "${incs}" "${deps}")

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

    set(deps ${ORACLE_LIBRARY})
    PDAL_ADD_PLUGIN(oci_writer_lib_name writer oci "${srcs}" "${incs}" "${deps}")

    #
    # OCI tests
    #
    if(BUILD_OCI_TESTS)
	set(srcs
	    test/OCITest.cpp
	)

        set(deps ${oci_reader_lib_name} ${oci_writer_lib_name})
	PDAL_ADD_TEST(ocitest "${srcs}" "${deps}")
    endif()
else()
    message(STATUS "Building without Oracle support")
endif()
