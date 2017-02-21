#
# curl support
#

find_package(Curl)
if (CURL_FOUND)
    set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
    find_package(Threads REQUIRED)
    include_directories(${CURL_INCLUDE_DIR})
    set(PDAL_ARBITER_ENABLED 1)
    add_definitions("-DARBITER_CURL")

    if (WIN32)
        add_definitions("-DWINDOWS")
    else()
        add_definitions("-DUNIX")
    endif()
endif()

