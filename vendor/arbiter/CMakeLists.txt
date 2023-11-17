#
# Make sure we don't attempt to add a library more than once
#

get_property(EXISTS GLOBAL PROPERTY _PDALARBITER_INCLUDED)
if (EXISTS)
    return()
endif()

PDAL_ADD_FREE_LIBRARY(${PDAL_ARBITER_LIB_NAME} STATIC arbiter.cpp)
target_include_directories(${PDAL_ARBITER_LIB_NAME}
    PRIVATE
        ${NLOHMANN_INCLUDE_DIR}
)

if (NOT TARGET CURL::libcurl)
    add_library(CURL::libcurl UNKNOWN IMPORTED)
    set_target_properties(CURL::libcurl
        PROPERTIES
            IMPORTED_LOCATION "${CURL_LIBRARIES}")
endif ()

target_link_libraries(${PDAL_ARBITER_LIB_NAME}
    PRIVATE
        CURL::libcurl
)
target_compile_definitions(${PDAL_ARBITER_LIB_NAME}
    PRIVATE
        ARBITER_CURL)

# Arbiter is built static but is included in a shared lib.
set_target_properties(${PDAL_ARBITER_LIB_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

if (WIN32)
    target_link_libraries(${PDAL_ARBITER_LIB_NAME} PRIVATE shlwapi)
    target_compile_definitions(${PDAL_ARBITER_LIB_NAME}
        PRIVATE
            ARBITER_DLL_EXPORT)
endif ()

# for dupenv_s
if (MINGW)
  target_link_libraries(${PDAL_ARBITER_LIB_NAME} PRIVATE ucrt)
endif ()

if (ZLIB_FOUND)
    target_compile_definitions(${PDAL_ARBITER_LIB_NAME}
        PRIVATE
            ARBITER_ZLIB)
endif()
if (OPENSSL_FOUND)
    target_include_directories(${PDAL_ARBITER_LIB_NAME}
        PRIVATE
            ${OPENSSL_INCLUDE_DIR})
    target_link_libraries(${PDAL_ARBITER_LIB_NAME}
        PRIVATE
            ${OPENSSL_LIBRARIES})
    target_compile_definitions(${PDAL_ARBITER_LIB_NAME}
        PRIVATE
            ARBITER_OPENSSL)
endif()

set_target_properties(${PDAL_ARBITER_LIB_NAME} PROPERTIES
    VERSION "${PDAL_BUILD_VERSION}"
    SOVERSION "${PDAL_API_VERSION}"
    CLEAN_DIRECT_OUTPUT 1)

set_property(GLOBAL PROPERTY _PDALARBITER_INCLUDED TRUE)

