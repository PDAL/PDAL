if (WIN32)
    include (${CMAKE_CURRENT_LIST_DIR}/win32_compiler_options.cmake)
else()
    include (${CMAKE_CURRENT_LIST_DIR}/unix_compiler_options.cmake)
endif()

