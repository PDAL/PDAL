#
# Make sure we don't attempt to add a library more than once
#

get_property(EXISTS GLOBAL PROPERTY _PDALARBITER_INCLUDED)
if (NOT PDAL_ARBITER_ENABLED OR EXISTS)
    return()
endif()

PDAL_ADD_FREE_LIBRARY(${PDAL_ARBITER_LIB_NAME} STATIC arbiter.cpp)
target_include_directories(${PDAL_ARBITER_LIB_NAME}
    PRIVATE
        ${PDAL_JSONCPP_INCLUDE_DIR}
        ${OPENSSL_INCLUDE_DIR})
target_link_libraries(${PDAL_ARBITER_LIB_NAME}
    PRIVATE
        ${CURL_LIBRARIES}
        ${OPENSSL_LIBRARIES}
        ${PDAL_JSONCPP_LIB_NAME})
target_compile_definitions(${PDAL_ARBITER_LIB_NAME}
    PRIVATE
        ARBITER_CURL
        ARBITER_OPENSSL)

if (MSVC)
    find_library(SHLWAPI Shlwapi.lib)
    target_link_libraries(${PDAL_ARBITER_LIB_NAME} PRIVATE ${SHLWAPI})
    target_compile_definitions(${PDAL_ARBITER_LIB_NAME}
        PRIVATE
            ARBITER_DLL_EXPORT)
else()
    #
    # Arbiter is built static but is included in a shared lib.
    #
    target_compile_options(${PDAL_ARBITER_LIB_NAME}
        PRIVATE
            -fPIC)
endif()
if (ZLIB_FOUND)
    target_compile_definitions(${PDAL_ARBITER_LIB_NAME}
        PRIVATE
            ARBITER_ZLIB)
endif()

set_target_properties(${PDAL_ARBITER_LIB_NAME} PROPERTIES
    VERSION "${PDAL_BUILD_VERSION}"
    SOVERSION "${PDAL_API_VERSION}"
    CLEAN_DIRECT_OUTPUT 1)

set_property(GLOBAL PROPERTY _PDALARBITER_INCLUDED TRUE)

