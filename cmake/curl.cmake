#
# curl support
#

find_package(CURL REQUIRED)
if (CURL_FOUND)
    set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
    find_package(Threads REQUIRED)
    include_directories(${CURL_INCLUDE_DIR})
    if (WIN32)
        add_definitions("-DWINDOWS")
    else()
        add_definitions("-DUNIX")
    endif()
else()
    message(FATAL_ERROR "Could not find curl")
endif()

