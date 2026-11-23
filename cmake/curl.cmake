#
# curl support
#

find_package(CURL REQUIRED)
if (CURL_FOUND)
    set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
    find_package(Threads REQUIRED)
    if (NOT TARGET CURL::libcurl)
        add_library(CURL::libcurl UNKNOWN IMPORTED)
        set_target_properties(CURL::libcurl
            PROPERTIES
                IMPORTED_LOCATION "${CURL_LIBRARIES}"
                INTERFACE_INCLUDE_DIRECTORIES "${CURL_INCLUDE_DIRS};${CURL_INCLUDE_DIR}")
    endif ()
    if (WIN32)
        set(PDAL_CURL_COMPILE_DEFINITIONS WINDOWS)
    else()
        set(PDAL_CURL_COMPILE_DEFINITIONS UNIX)
    endif()
else()
    message(FATAL_ERROR "Could not find curl")
endif()

