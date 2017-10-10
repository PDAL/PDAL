#
# arbiter support
#

include(${PDAL_CMAKE_DIR}/curl.cmake)

if (CURL_FOUND)
    set(PDAL_ARBITER_ENABLED 1)
    set(PDAL_ARBITER_LIB_NAME pdal_arbiter)
endif()

