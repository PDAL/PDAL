if (MSVC)
    # Since tests link CRT dynamically (/MD[d]), require gtest to
    #link dynamically too (default is /MT[d])
    option(gtest_force_shared_crt "Always use shared Visual C++ run-time DLL" ON)
endif()
add_subdirectory(vendor/gtest)
